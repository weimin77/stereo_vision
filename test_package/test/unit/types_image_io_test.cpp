#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "sv_image_io.hpp"
#include "sv_types.hpp"

namespace {

namespace fs = std::filesystem;

constexpr double kPngDisparityTolerance = 1.0 / 32.0;
constexpr double kDepthPngTolerance = 1.0 / 2000.0;

fs::path temporary_path(const std::string& name) {
    const fs::path dir = fs::temp_directory_path() / "stereo_vision_tests";
    fs::create_directories(dir);
    return dir / name;
}

sv::Image make_image(int width, int height, int channels, std::uint8_t value) {
    sv::Image image;
    image.width = width;
    image.height = height;
    image.channels = channels;
    image.data.assign(static_cast<std::size_t>(width) * height * channels, value);
    return image;
}

}  // namespace

TEST(Types, ImageValidityRejectsInconsistentSizes) {
    sv::Image image = make_image(4, 3, 1, 7);
    EXPECT_TRUE(sv::image_valid(image));

    image.data.pop_back();
    EXPECT_FALSE(sv::image_valid(image));

    sv::Image empty;
    EXPECT_FALSE(sv::image_valid(empty));

    sv::Image odd = make_image(2, 2, 2, 0);
    EXPECT_FALSE(sv::image_valid(odd));
}

TEST(Types, GrayConversionKeepsGrayInput) {
    const sv::Image gray = make_image(2, 2, 1, 42);
    sv::Image converted;
    sv::image_to_gray(gray, converted);
    EXPECT_EQ(converted.data, gray.data);
    EXPECT_EQ(converted.channels, 1);
}

TEST(Types, GrayConversionMatchesBgrWeights) {
    sv::Image color = make_image(1, 1, 3, 0);
    color.data = {0, 0, 255};  // BGR, pure red
    sv::Image gray;
    sv::image_to_gray(color, gray);
    ASSERT_EQ(gray.data.size(), 1U);
    EXPECT_NEAR(gray.data[0], 76.0, 2.0);
}

TEST(Types, PercentilesOfFiniteSamples) {
    const std::vector<float> values = {1.0F, 2.0F, 3.0F, 4.0F, 5.0F};
    double out[3] = {0.0, 0.0, 0.0};
    sv::percentiles(values, out);
    EXPECT_NEAR(out[1], 3.0, 1e-6);
    EXPECT_LT(out[0], out[1]);
    EXPECT_GT(out[2], out[1]);
}

TEST(Types, PercentilesIgnoreNonFiniteSamples) {
    const std::vector<float> values = {std::numeric_limits<float>::quiet_NaN(), 4.0F, 4.0F, 4.0F};
    double out[3] = {0.0, 0.0, 0.0};
    sv::percentiles(values, out);
    EXPECT_NEAR(out[0], 4.0, 1e-6);
    EXPECT_NEAR(out[1], 4.0, 1e-6);
    EXPECT_NEAR(out[2], 4.0, 1e-6);
}

TEST(ImageIo, RejectsUnsupportedInputExtension) {
    EXPECT_FALSE(sv::image_path_supported("frame.tif"));
    EXPECT_TRUE(sv::image_path_supported("frame.JPG"));
    EXPECT_TRUE(sv::image_path_supported("frame.bmp"));

    sv::Image image;
    std::string error;
    EXPECT_FALSE(sv::image_load("frame.tif", image, &error));
    EXPECT_FALSE(error.empty());
}

TEST(ImageIo, PngRoundTripKeepsPixels) {
    const sv::Image source = make_image(8, 5, 3, 128);
    const fs::path path = temporary_path("roundtrip.png");
    std::string error;
    ASSERT_TRUE(sv::image_save_png(path.string(), source, &error)) << error;

    sv::Image loaded;
    ASSERT_TRUE(sv::image_load(path.string(), loaded, &error)) << error;
    EXPECT_EQ(loaded.width, source.width);
    EXPECT_EQ(loaded.height, source.height);
    EXPECT_EQ(loaded.channels, 3);
    EXPECT_EQ(loaded.data, source.data);
    fs::remove(path);
}

TEST(ImageIo, DisparityPngUsesFixedPointScale) {
    sv::DisparityImage disparity;
    disparity.width = 3;
    disparity.height = 1;
    disparity.data = {12.5F, std::numeric_limits<float>::quiet_NaN(), 0.0F};

    const fs::path path = temporary_path("disparity.png");
    std::string error;
    ASSERT_TRUE(sv::disparity_save(path.string(), disparity, &error)) << error;

    // image_load() narrows to 8 bits by design, so the 16-bit sample is read with OpenCV.
    const cv::Mat loaded = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    ASSERT_FALSE(loaded.empty());
    ASSERT_EQ(loaded.type(), CV_16UC1);
    EXPECT_NEAR(static_cast<double>(loaded.at<std::uint16_t>(0, 0)) / sv::kDisparityPngScale, 12.5,
                kPngDisparityTolerance);
    EXPECT_EQ(loaded.at<std::uint16_t>(0, 1), 0);
    fs::remove(path);
}

TEST(ImageIo, RejectsUnsupportedDisparityExtension) {
    sv::DisparityImage disparity;
    disparity.width = 1;
    disparity.height = 1;
    disparity.data = {1.0F};

    std::string error;
    EXPECT_FALSE(sv::disparity_save(temporary_path("disparity.bmp").string(), disparity, &error));
    EXPECT_FALSE(error.empty());
}

TEST(ImageIo, DepthPngUsesMillimetres) {
    sv::DepthImage depth;
    depth.width = 2;
    depth.height = 1;
    depth.data = {2.5F, -1.0F};

    const fs::path path = temporary_path("depth.png");
    std::string error;
    ASSERT_TRUE(sv::depth_save(path.string(), depth, &error)) << error;

    const cv::Mat loaded = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    ASSERT_FALSE(loaded.empty());
    ASSERT_EQ(loaded.type(), CV_16UC1);
    EXPECT_NEAR(static_cast<double>(loaded.at<std::uint16_t>(0, 0)) / sv::kDepthPngScale, 2.5, kDepthPngTolerance);
    EXPECT_EQ(loaded.at<std::uint16_t>(0, 1), 0);
    fs::remove(path);
}

TEST(ImageIo, MissingDirectoryReportsError) {
    const sv::Image image = make_image(2, 2, 1, 1);
    std::string error;
    EXPECT_FALSE(sv::image_save_png("/nonexistent-directory-for-test/frame.png", image, &error));
    EXPECT_NE(error.find("does not exist"), std::string::npos) << error;
    EXPECT_FALSE(fs::exists("/nonexistent-directory-for-test/frame.png"));
}
