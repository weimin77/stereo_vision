#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

#include "sv_config.hpp"
#include "sv_pipeline.hpp"
#include "sv_types.hpp"

namespace {

/// Medians of the scene's disparity_gt_left.npy / depth_gt_left.npy, scaled to the 640x480 pair.
constexpr double kSceneDisparityP50 = 73.26;
constexpr double kSceneDepthP50 = 0.1129;
constexpr double kDisparityTolerance = 3.0;
constexpr double kDepthTolerance = 0.005;

// test_package/conanfile.py copies resources/ next to the build and defines RESOURCES_PATH,
// so this program never depends on the directory it is started from.
std::string resource(const std::string& name) { return std::string(RESOURCES_PATH) + "/oven/" + name; }

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

}  // namespace

int main() {
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
    if (!pipeline.run()) {
        std::cerr << "example: " << pipeline.last_error() << '\n';
        return 1;
    }

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

    return save_artifacts(pipeline, "example_output") ? 0 : 1;
}
