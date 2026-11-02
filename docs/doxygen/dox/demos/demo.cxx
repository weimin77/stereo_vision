// demo.cxx
//
// End-to-end use of the stereo pipeline. Build it after the package is installed; the only
// headers needed are the public ones from include/.
#include <cstdio>
#include <string>

#include "sv_config.hpp"
#include "sv_image_io.hpp"
#include "sv_pipeline.hpp"
#include "sv_types.hpp"

int main(int argc, char** argv) {
    if (argc < 4) {
        std::printf("usage: %s <left> <right> <config.json>\n", argv[0]);
        return 1;
    }

    // 1. Configuration: matcher parameters plus the calibration paths. No images here.
    sv::Config config;
    std::string error;
    if (!sv::config_load(argv[3], config, &error)) {
        std::fprintf(stderr, "config: %s\n", error.c_str());
        return 1;
    }

    sv::Pipeline pipeline{config};

    // 2. Calibration: stereo.yml is mandatory, the per-camera files override M/D when given.
    if (!pipeline.set_calibration_files(config.stereo_yml, config.left_mono_yml, config.right_mono_yml)) {
        std::fprintf(stderr, "calibration: %s\n", pipeline.last_error().c_str());
        return 1;
    }

    // 3. Input images: the pipeline reads jpg, jpeg, png and bmp, or takes pixel buffers directly.
    if (!pipeline.set_input_image(argv[1], argv[2])) {
        std::fprintf(stderr, "input: %s\n", pipeline.last_error().c_str());
        return 1;
    }

    // 4. Compute. This step writes nothing, so a caller can inspect the result before saving.
    if (!pipeline.run()) {
        std::fprintf(stderr, "run: %s\n", pipeline.last_error().c_str());
        return 1;
    }

    const sv::RunReport& report = pipeline.report();
    std::printf("rectified %dx%d, valid %.1f%%, disparity p50 %.2f px, depth p50 %.3f m\n", report.rectified_width,
                report.rectified_height, report.valid_ratio * 100.0, report.disparity_p50, report.depth_p50);

    // 5. Results are read by reference: the point cloud can be hundreds of megabytes.
    const sv::DisparityImage& disparity = pipeline.get_disparity_image();
    const sv::DepthImage& depth = pipeline.get_depth_image();
    const sv::PointCloud& cloud = pipeline.get_pointcloud();
    std::printf("disparity %dx%d, depth %dx%d, cloud %zu points (coloured: %s)\n", disparity.width, disparity.height,
                depth.width, depth.height, cloud.points.size(), cloud.colored ? "yes" : "no");

    // 6. Saving is explicit: png is 16-bit fixed point, tif is 32-bit float, pcd/ply follow the extension.
    pipeline.save_disparity_image("disparity.png");
    pipeline.save_depth_image("depth.png");
    // A 16-bit or millimetre png reads as a near-black frame in most viewers; colorize=true writes
    // an 8-bit ramp instead, for looking at rather than for measuring.
    pipeline.save_disparity_image("disparity_color.png", true);
    pipeline.save_depth_image("depth_color.png", true);
    pipeline.save_pointcloud("cloud.pcd");
    pipeline.save_pointcloud("cloud.ply");
    return 0;
}
