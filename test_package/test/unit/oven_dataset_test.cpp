#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "sv_calibration.hpp"
#include "sv_config.hpp"
#include "sv_pipeline.hpp"

namespace {

/// The BlenderProc oven scene shipped in resources/oven, rendered at 640x480 from a 320x240 rig.
/// The reference values below are the medians of disparity_gt_left.npy / depth_gt_left.npy after
/// scaling the 320-pixel ground truth to the 640-pixel pair.
constexpr double kSceneFocalPx = 140.0;
constexpr double kSceneBaselineM = 0.06;
constexpr double kSceneGroundTruthDisparityPx = 73.26;
constexpr double kSceneGroundTruthDepthM = 0.11286;
constexpr double kDisparityTolerancePx = 3.0;
constexpr double kDepthToleranceM = 0.005;

std::string resource(const std::string& name) { return std::string(RESOURCES_PATH) + "/oven/" + name; }

sv::Pipeline oven_pipeline() {
    sv::Config config;
    std::string error;
    EXPECT_TRUE(sv::config_load(resource("sgbm_config.json"), config, &error)) << error;

    sv::Pipeline pipeline{config};
    EXPECT_TRUE(pipeline.set_calibration_files(resource("stereo.yml"), resource("left_mono.yml"),
                                               resource("right_mono.yml")))
        << pipeline.last_error();
    EXPECT_TRUE(pipeline.set_input_image(resource("left_on.png"), resource("right_on.png")))
        << pipeline.last_error();
    return pipeline;
}

}  // namespace

TEST(Oven, CalibrationCarriesTheScaledSceneIntrinsics) {
    sv::Calibration calibration;
    std::string error;
    ASSERT_TRUE(sv::calibration_load_stereo(resource("stereo.yml"), calibration, &error)) << error;
    ASSERT_TRUE(sv::calibration_override_left(resource("left_mono.yml"), calibration, &error)) << error;
    ASSERT_TRUE(sv::calibration_override_right(resource("right_mono.yml"), calibration, &error)) << error;

    EXPECT_NEAR(calibration.m1[0], kSceneFocalPx, 1e-9);
    EXPECT_NEAR(calibration.m2[0], kSceneFocalPx, 1e-9);
    EXPECT_NEAR(calibration.m1[2], 319.5, 1e-9);
    EXPECT_NEAR(calibration.m1[5], 239.5, 1e-9);
    EXPECT_NEAR(calibration.t[0], -kSceneBaselineM, 1e-9);
}

TEST(Oven, ConfigDerivesTheSearchWindowFromTheWorkingDepth) {
    sv::Config config;
    std::string error;
    ASSERT_TRUE(sv::config_load(resource("sgbm_config.json"), config, &error)) << error;

    sv::RectifyGeometry geometry;
    geometry.fx = kSceneFocalPx;
    geometry.baseline = kSceneBaselineM;
    geometry.focal_baseline = kSceneFocalPx * kSceneBaselineM;

    sv::SgbmParams params;
    ASSERT_TRUE(sv::config_resolve_window(geometry, config, params, &error)) << error;

    const double near_disparity = geometry.focal_baseline / config.working_near_m;
    EXPECT_LE(static_cast<double>(params.min_disparity), geometry.focal_baseline / config.working_far_m);
    EXPECT_GE(static_cast<double>(params.min_disparity + params.num_disparities), near_disparity);
}

TEST(Oven, PipelineReproducesTheSceneGroundTruth) {
    sv::Pipeline pipeline = oven_pipeline();
    ASSERT_TRUE(pipeline.run()) << pipeline.last_error();

    const sv::RunReport& report = pipeline.report();
    EXPECT_GT(report.valid_ratio, 0.5);
    EXPECT_NEAR(report.disparity_p50, kSceneGroundTruthDisparityPx, kDisparityTolerancePx);
    EXPECT_NEAR(report.depth_p50, kSceneGroundTruthDepthM, kDepthToleranceM);
    EXPECT_FALSE(pipeline.get_pointcloud().points.empty());
}
