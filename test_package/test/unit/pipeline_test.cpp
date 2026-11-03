#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "sv_config.hpp"
#include "sv_image_io.hpp"
#include "sv_pipeline.hpp"
#include "synthetic.hpp"

namespace {

namespace fs = std::filesystem;

constexpr int kKnownShift = 20;
constexpr int kNumDisparities = 64;
constexpr double kDisparityTolerance = 0.5;
constexpr double kDepthTolerance = 0.02;

fs::path temporary_path(const std::string& name) {
    const fs::path dir = fs::temp_directory_path() / "stereo_vision_pipeline_tests";
    fs::create_directories(dir);
    return dir / name;
}

std::string resource(const std::string& name) { return std::string(RESOURCES_PATH) + "/" + name; }

sv::Config test_config() {
    sv::Config config = sv::config_default();
    config.matcher.num_disparities = kNumDisparities;
    config.matcher.mode = sv::SgbmMode::Sgbm;
    config.rectify.crop_to_common_roi = false;
    return config;
}

/// A pipeline holding the synthetic rig and the shifted texture pair.
sv::Pipeline make_pipeline() {
    sv::Pipeline pipeline{test_config()};
    const sv::Image left = synthetic::blocky_texture(synthetic::kTestWidth, synthetic::kTestHeight, 23U);
    const sv::Image right = synthetic::shift_horizontal(left, kKnownShift);
    pipeline.set_calibration(synthetic::pinhole_rig());
    pipeline.set_input_image(left, right);
    return pipeline;
}

}  // namespace

TEST(Pipeline, RunWithoutInputFails) {
    sv::Pipeline pipeline{test_config()};
    EXPECT_FALSE(pipeline.run());
    EXPECT_NE(pipeline.last_error().find("set_input_image"), std::string::npos) << pipeline.last_error();
}

TEST(Pipeline, RunWithoutCalibrationFails) {
    sv::Pipeline pipeline{test_config()};
    pipeline.set_input_image(synthetic::blocky_texture(64, 48, 1U), synthetic::blocky_texture(64, 48, 1U));
    EXPECT_FALSE(pipeline.run());
    EXPECT_NE(pipeline.last_error().find("calibration"), std::string::npos) << pipeline.last_error();
}

TEST(Pipeline, ResultsAreUnavailableBeforeRun) {
    sv::Pipeline pipeline{test_config()};
    EXPECT_THROW(pipeline.get_disparity_image(), std::logic_error);
    EXPECT_THROW(pipeline.get_depth_image(), std::logic_error);
    EXPECT_THROW(pipeline.get_pointcloud(), std::logic_error);
}

TEST(Pipeline, EndToEndRecoversDisparityAndDepth) {
    sv::Pipeline pipeline = make_pipeline();
    ASSERT_TRUE(pipeline.run()) << pipeline.last_error();

    const sv::DisparityImage& disparity = pipeline.get_disparity_image();
    const sv::DepthImage& depth = pipeline.get_depth_image();
    EXPECT_EQ(disparity.width, synthetic::kTestWidth);
    EXPECT_EQ(disparity.height, synthetic::kTestHeight);

    const double median_disparity = synthetic::median_of_finite(disparity.data);
    EXPECT_NEAR(median_disparity, static_cast<double>(kKnownShift), kDisparityTolerance);

    const double median_depth = synthetic::median_of_finite(depth.data);
    const double expected_depth = (synthetic::kTestFocal * synthetic::kTestBaseline) / kKnownShift;
    EXPECT_NEAR(median_depth, expected_depth, kDepthTolerance);

    const sv::RunReport& report = pipeline.report();
    EXPECT_GT(report.valid_ratio, 0.2);
    EXPECT_EQ(report.num_disparities, kNumDisparities);
    EXPECT_GE(report.rectify_ms, 0.0);
}

TEST(Pipeline, PointCloudIsCachedUntilTheNextRun) {
    sv::Pipeline pipeline = make_pipeline();
    ASSERT_TRUE(pipeline.run()) << pipeline.last_error();

    const sv::PointCloud& first = pipeline.get_pointcloud();
    const sv::PointCloud& second = pipeline.get_pointcloud();
    EXPECT_EQ(&first, &second);
    EXPECT_FALSE(first.points.empty());

    ASSERT_TRUE(pipeline.run()) << pipeline.last_error();
    const sv::PointCloud& third = pipeline.get_pointcloud();
    EXPECT_FALSE(third.points.empty());
}

TEST(Pipeline, ChangingTheInputInvalidatesTheResults) {
    sv::Pipeline pipeline = make_pipeline();
    ASSERT_TRUE(pipeline.run()) << pipeline.last_error();

    pipeline.set_input_image(synthetic::blocky_texture(64, 48, 2U), synthetic::blocky_texture(64, 48, 2U));
    EXPECT_THROW(pipeline.get_disparity_image(), std::logic_error);
}

TEST(Pipeline, FailedImageLoadKeepsThePreviousPair) {
    sv::Pipeline pipeline = make_pipeline();
    EXPECT_FALSE(pipeline.set_input_image(resource("missing.png"), resource("missing.png")));
    EXPECT_FALSE(pipeline.last_error().empty());
    EXPECT_TRUE(pipeline.run()) << pipeline.last_error();
}

TEST(Pipeline, MismatchedImageSizesAreRejected) {
    sv::Pipeline pipeline{test_config()};
    const sv::Image left = synthetic::blocky_texture(64, 48, 3U);
    const sv::Image right = synthetic::blocky_texture(32, 24, 3U);
    EXPECT_FALSE(pipeline.set_input_image(left, right));
    EXPECT_NE(pipeline.last_error().find("same size"), std::string::npos) << pipeline.last_error();
}

TEST(Pipeline, SavesEveryArtefact) {
    sv::Pipeline pipeline = make_pipeline();
    ASSERT_TRUE(pipeline.run()) << pipeline.last_error();

    const fs::path disparity_path = temporary_path("disparity.png");
    const fs::path depth_path = temporary_path("depth.tif");
    const fs::path pcd_path = temporary_path("cloud.pcd");
    const fs::path ply_path = temporary_path("cloud.ply");

    EXPECT_TRUE(pipeline.save_disparity_image(disparity_path.string())) << pipeline.last_error();
    EXPECT_TRUE(pipeline.save_depth_image(depth_path.string())) << pipeline.last_error();
    EXPECT_TRUE(pipeline.save_pointcloud(pcd_path.string())) << pipeline.last_error();
    EXPECT_TRUE(pipeline.save_pointcloud(ply_path.string())) << pipeline.last_error();

    EXPECT_TRUE(fs::exists(disparity_path));
    EXPECT_TRUE(fs::exists(depth_path));
    EXPECT_GT(fs::file_size(pcd_path), 0U);
    EXPECT_GT(fs::file_size(ply_path), 0U);

    fs::remove(disparity_path);
    fs::remove(depth_path);
    fs::remove(pcd_path);
    fs::remove(ply_path);
}

TEST(Pipeline, SaveRejectsAnUnknownPointCloudExtension) {
    sv::Pipeline pipeline = make_pipeline();
    ASSERT_TRUE(pipeline.run()) << pipeline.last_error();
    EXPECT_FALSE(pipeline.save_pointcloud(temporary_path("cloud.bmp").string()));
    EXPECT_NE(pipeline.last_error().find(".pcd"), std::string::npos) << pipeline.last_error();
}

TEST(Pipeline, SaveReportsAMissingDirectory) {
    sv::Pipeline pipeline = make_pipeline();
    ASSERT_TRUE(pipeline.run()) << pipeline.last_error();
    EXPECT_FALSE(pipeline.save_disparity_image("/nonexistent-directory-for-test/disparity.png"));
    EXPECT_FALSE(fs::exists("/nonexistent-directory-for-test/disparity.png"));
}

TEST(Pipeline, CalibrationFilesLoadFromResources) {
    sv::Pipeline pipeline{test_config()};
    ASSERT_TRUE(pipeline.set_calibration_files(resource("stereo.yml"), resource("left_mono.yml"),
                                               resource("right_mono.yml")))
        << pipeline.last_error();
    EXPECT_TRUE(pipeline.last_error().empty());
}

TEST(Config, LoadsTheSampleConfiguration) {
    sv::Config config;
    std::string error;
    ASSERT_TRUE(sv::config_load(resource("sgbm_config.json"), config, &error)) << error;

    EXPECT_EQ(config.profile, "fast");
    EXPECT_EQ(config.matcher.block_size, 5);
    EXPECT_EQ(config.matcher.mode, sv::SgbmMode::ThreeWay);
    EXPECT_EQ(config.matcher.num_disparities, 0);
    EXPECT_NEAR(config.working_near_m, 0.07, 1e-9);
    EXPECT_NEAR(config.working_far_m, 0.25, 1e-9);
    EXPECT_EQ(config.stereo_yml, "stereo.yml");
    EXPECT_EQ(config.left_mono_yml, "left_mono.yml");
    EXPECT_EQ(config.right_mono_yml, "right_mono.yml");
    EXPECT_EQ(config.output_prefix, "oven");
    EXPECT_TRUE(config.cloud_binary);
    EXPECT_FALSE(config.cloud_keep_invalid);
}

TEST(Config, ResolvesTheWindowFromTheWorkingDepth) {
    sv::Config config;
    std::string error;
    ASSERT_TRUE(sv::config_load(resource("sgbm_config.json"), config, &error)) << error;

    sv::RectifyGeometry geometry;
    geometry.fx = synthetic::kTestFocal;
    geometry.baseline = synthetic::kTestBaseline;
    geometry.focal_baseline = synthetic::kTestFocal * synthetic::kTestBaseline;

    sv::SgbmParams params;
    ASSERT_TRUE(sv::config_resolve_window(geometry, config, params, &error)) << error;

    const double near_disparity = geometry.focal_baseline / config.working_near_m;
    EXPECT_EQ(params.num_disparities % sv::kDisparityStep, 0);
    EXPECT_LE(static_cast<double>(params.min_disparity), geometry.focal_baseline / config.working_far_m);
    EXPECT_GE(static_cast<double>(params.min_disparity + params.num_disparities), near_disparity);
}

TEST(Config, RejectsAMalformedConfiguration) {
    const fs::path path = temporary_path("malformed.json");
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    out << "{ \"block_size\": 4 }";
    out.close();

    sv::Config config;
    std::string error;
    EXPECT_FALSE(sv::config_load(path.string(), config, &error));
    EXPECT_FALSE(error.empty());
    fs::remove(path);
}
