#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "sv_config.hpp"
#include "sv_pipeline.hpp"
#include "sv_types.hpp"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <psapi.h>
#if defined(_MSC_VER)
// GetProcessMemoryInfo lives in kernel32 when PSAPI_VERSION >= 2, but psapi.lib is in every SDK,
// so naming it removes the dependency on which version the toolchain defaults to.
#pragma comment(lib, "psapi.lib")
#endif
#else
#include <sys/resource.h>
#endif

namespace {

/// Medians of the scene's disparity_gt_left.npy / depth_gt_left.npy, scaled to the 640x480 pair.
constexpr double kSceneDisparityP50 = 73.26;
constexpr double kSceneDepthP50 = 0.1129;
constexpr double kDisparityTolerance = 3.0;
constexpr double kDepthTolerance = 0.005;

using Clock = std::chrono::steady_clock;

constexpr double kKib = 1024.0;
constexpr double kMib = kKib * kKib;

/// Milliseconds between two readings of the monotonic clock.
double elapsed_ms(Clock::time_point from, Clock::time_point to) {
    return std::chrono::duration<double, std::milli>(to - from).count();
}

/**
 * @brief [en] Peak resident set size of this process in MiB, or a negative value when the platform
 * cannot report it
 * @brief [zh] 本进程的峰值常驻内存（MiB），平台无法报告时返回负值
 *
 * Every platform is read through a mechanism that starts at zero in this process. `getrusage` is
 * deliberately avoided on Linux: its `ru_maxrss` is inherited across `fork` and not reset by
 * `exec`, so a program started from a heavy parent (conan's Python, an IDE) reports the parent's
 * high-water mark as its own — measured here as 72 MB against a true 33 MB. `/proc/self/status`
 * `VmHWM` is reset by `exec`, so it reports this process only.
 */
double peak_resident_mb() {
#if defined(_WIN32)
    // PeakWorkingSetSize belongs to this process object and is created fresh by spawn.
    PROCESS_MEMORY_COUNTERS_EX counters{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters),
                             sizeof(counters)) == 0) {
        return -1.0;
    }
    return static_cast<double>(counters.PeakWorkingSetSize) / kMib;
#elif defined(__APPLE__)
    // BSD resets the resource usage on exec, so ru_maxrss here is this process's own peak.
    struct rusage usage {};
    if (getrusage(RUSAGE_SELF, &usage) != 0) {
        return -1.0;
    }
    return static_cast<double>(usage.ru_maxrss) / kMib; // bytes on macOS
#else
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        if (line.rfind("VmHWM:", 0) != 0) {
            continue;
        }
        const std::size_t first = line.find_first_of("0123456789");
        if (first == std::string::npos) {
            return -1.0;
        }
        return static_cast<double>(std::stol(line.substr(first))) / kKib; // reported in kB
    }
    return -1.0;
#endif
}

// test_package/conanfile.py copies resources/ next to the build and defines RESOURCES_PATH,
// so this program never depends on the directory it is started from.
std::string resource(const std::string& name) { return std::string(RESOURCES_PATH) + "/" + name; }

bool load_config(sv::Config& config) {
    std::string error;
    if (!sv::config_load(resource("sgbm_config.json"), config, &error)) {
        std::cerr << "example: configuration rejected: " << error << '\n';
        return false;
    }
    for (const std::string& warning : config.warnings) {
        std::cerr << "example: configuration warning: " << warning << '\n';
    }
    return true;
}

void print_report(const sv::Pipeline& pipeline) {
    const sv::RunReport& report = pipeline.report();
    std::cout << "rectified " << report.rectified_width << "x" << report.rectified_height << '\n'
              << "window    " << report.min_disparity << " + " << report.num_disparities << '\n'
              << "timing    rectify " << report.rectify_ms << " ms, match " << report.match_ms << " ms, depth "
              << report.depth_ms << " ms\n"
              << "valid     " << report.valid_ratio * 100.0 << " %\n"
              << "disparity p05/p50/p95 " << report.disparity_p05 << " / " << report.disparity_p50 << " / "
              << report.disparity_p95 << " px\n"
              << "depth     p05/p50/p95 " << report.depth_p05 << " / " << report.depth_p50 << " / " << report.depth_p95
              << " m\n";
}

bool save_artifacts(const sv::Pipeline& pipeline, const std::filesystem::path& out_dir) {
    std::error_code code;
    std::filesystem::create_directories(out_dir, code);
    if (code) {
        std::cerr << "example: cannot create " << out_dir << ": " << code.message() << '\n';
        return false;
    }

    // Two artefacts per map: the plain one carries the measurement, the _color one is what a
    // viewer can actually show. Both come from the same result, so they cannot disagree.
    const std::filesystem::path disparity = out_dir / "disparity.png";
    const std::filesystem::path disparity_color = out_dir / "disparity_color.png";
    const std::filesystem::path depth = out_dir / "depth.png";
    const std::filesystem::path depth_color = out_dir / "depth_color.png";
    const std::filesystem::path cloud = out_dir / "cloud.pcd";

    const bool saved = pipeline.save_disparity_image(disparity.string()) &&
                       pipeline.save_depth_image(depth.string()) &&
                       pipeline.save_disparity_image(disparity_color.string(), true) &&
                       pipeline.save_depth_image(depth_color.string(), true) &&
                       pipeline.save_pointcloud(cloud.string());
    if (!saved) {
        std::cerr << "example: " << pipeline.last_error() << '\n';
        return false;
    }

    std::cout << "saved     " << disparity.filename().string() << ", " << depth.filename().string()
              << " (measurement) and " << disparity_color.filename().string() << ", "
              << depth_color.filename().string() << " (colour), plus " << cloud.filename().string() << " in "
              << out_dir.string() << '\n';
    return true;
}

/**
 * @brief Wall-clock cost of the run, printed last so a reader sees the result before the bill.
 *
 * Three phases, because they answer different questions: `load` is I/O and depends on the storage,
 * `run` is the algorithm the README benchmarks, and `write` grows with whatever the caller saves.
 * The per-stage numbers inside `run` come from sv::RunReport; these frame them.
 */
void print_cost(Clock::time_point start, Clock::time_point loaded, Clock::time_point ran, Clock::time_point written) {
    std::cout << std::fixed << std::setprecision(1) << "elapsed   load " << elapsed_ms(start, loaded) << " ms, run "
              << elapsed_ms(loaded, ran) << " ms, write " << elapsed_ms(ran, written) << " ms, total "
              << elapsed_ms(start, written) << " ms\n";

    // Peak, not current: getrusage reports the high-water mark, so reading it now covers the
    // whole process, including the buffers the writers have already released.
    const double peak_mb = peak_resident_mb();
    if (peak_mb < 0.0) {
        std::cout << "memory    peak unavailable on this platform\n";
    } else {
        std::cout << "memory    peak " << peak_mb << " MB resident\n";
    }
    std::cout.unsetf(std::ios::floatfield);
}

}  // namespace

int main() {
    const Clock::time_point start = Clock::now();

    sv::Config config = sv::config_default();
    if (!load_config(config)) {
        return 1;
    }

    sv::Pipeline pipeline{config};

    // The calibration is injected by absolute path because the paths written inside
    // sgbm_config.json are resolved against the process working directory.
    if (!pipeline.set_calibration_files(resource("stereo.yml"), resource("left_mono.yml"),
                                        resource("right_mono.yml"))) {
        std::cerr << "example: " << pipeline.last_error() << '\n';
        return 1;
    }
    if (!pipeline.set_input_image(resource("left_on.png"), resource("right_on.png"))) {
        std::cerr << "example: " << pipeline.last_error() << '\n';
        return 1;
    }
    const Clock::time_point loaded = Clock::now();

    if (!pipeline.run()) {
        std::cerr << "example: " << pipeline.last_error() << '\n';
        return 1;
    }
    const Clock::time_point ran = Clock::now();

    print_report(pipeline);
    std::cout << "cloud     " << pipeline.get_pointcloud().points.size() << " points\n";

    // The pair is the BlenderProc oven scene: 140 px focal length, 60 mm baseline. Its own
    // ground truth gives a median disparity of 73.26 px, i.e. 0.1129 m in front of the left
    // camera, so a wrong focal length, baseline or principal point shows up here at once.
    const sv::RunReport& report = pipeline.report();
    if (!(report.valid_ratio > 0.5) || std::abs(report.disparity_p50 - kSceneDisparityP50) > kDisparityTolerance ||
        std::abs(report.depth_p50 - kSceneDepthP50) > kDepthTolerance) {
        std::cerr << "example: the result is outside the range the scene ground truth allows\n";
        return 1;
    }

    if (!save_artifacts(pipeline, "example_output")) {
        return 1;
    }
    print_cost(start, loaded, ran, Clock::now());
    return 0;
}
