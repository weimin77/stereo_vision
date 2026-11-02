#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

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

TEST(ImageIo, ColorizedDisparityIsAnEightBitRampWithBlackGaps) {
    // A ramp plus one hole: the ramp must stay monotonic, the hole must stay black.
    sv::DisparityImage disparity;
    disparity.width = 4;
    disparity.height = 1;
    disparity.data = {10.0F, 20.0F, std::numeric_limits<float>::quiet_NaN(), 40.0F};

    const fs::path path = temporary_path("disparity_color.png");
    std::string error;
    ASSERT_TRUE(sv::disparity_save(path.string(), disparity, &error, true)) << error;

    const cv::Mat loaded = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    ASSERT_FALSE(loaded.empty());
    EXPECT_EQ(loaded.type(), CV_8UC3);
    EXPECT_EQ(loaded.cols, disparity.width);
    EXPECT_EQ(loaded.rows, disparity.height);
    EXPECT_EQ(loaded.at<cv::Vec3b>(0, 2), cv::Vec3b(0, 0, 0));

    // Different samples land on different ramp entries; the exact ordering is pinned by the ramp test.
    EXPECT_NE(loaded.at<cv::Vec3b>(0, 0), loaded.at<cv::Vec3b>(0, 3));
    fs::remove(path);
}

TEST(ImageIo, ColorizedDepthKeepsNearWarm) {
    sv::DepthImage depth;
    depth.width = 2;
    depth.height = 1;
    depth.data = {0.1F, 2.0F};

    const fs::path path = temporary_path("depth_color.png");
    std::string error;
    ASSERT_TRUE(sv::depth_save(path.string(), depth, &error, true)) << error;

    const cv::Mat loaded = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    ASSERT_FALSE(loaded.empty());
    ASSERT_EQ(loaded.type(), CV_8UC3);
    // Depth grows away from the camera, so the ramp is inverted: the near sample is the red end.
    const cv::Vec3b near_pixel = loaded.at<cv::Vec3b>(0, 0);
    const cv::Vec3b far_pixel = loaded.at<cv::Vec3b>(0, 1);
    EXPECT_GT(static_cast<int>(near_pixel[2]), static_cast<int>(near_pixel[0]));
    EXPECT_GT(static_cast<int>(far_pixel[0]), static_cast<int>(far_pixel[2]));
    fs::remove(path);
}

TEST(ImageIo, ColorizedOutputRejectsTheFloatTiff) {
    sv::DepthImage depth;
    depth.width = 1;
    depth.height = 1;
    depth.data = {1.0F};

    std::string error;
    EXPECT_FALSE(sv::depth_save(temporary_path("depth_color.tif").string(), depth, &error, true));
    EXPECT_NE(error.find("colorize"), std::string::npos) << error;
}

TEST(ImageIo, ColorizeLeavesTheMeasurementEncodingUntouched) {
    sv::DisparityImage disparity;
    disparity.width = 2;
    disparity.height = 1;
    disparity.data = {12.5F, 40.0F};

    const fs::path plain = temporary_path("disparity_plain.png");
    const fs::path colored = temporary_path("disparity_colored.png");
    std::string error;
    ASSERT_TRUE(sv::disparity_save(plain.string(), disparity, &error)) << error;
    ASSERT_TRUE(sv::disparity_save(colored.string(), disparity, &error, true)) << error;

    // The default stays 16-bit fixed point: colorize is opt-in, not a change of meaning.
    const cv::Mat plain_mat = cv::imread(plain.string(), cv::IMREAD_UNCHANGED);
    ASSERT_FALSE(plain_mat.empty());
    EXPECT_EQ(plain_mat.type(), CV_16UC1);
    const cv::Mat colored_mat = cv::imread(colored.string(), cv::IMREAD_UNCHANGED);
    ASSERT_FALSE(colored_mat.empty());
    EXPECT_EQ(colored_mat.type(), CV_8UC3);
    fs::remove(plain);
    fs::remove(colored);
}

TEST(ImageIo, ColorizedRampSaturatesAtTheRobustRange) {
    // A uniform 10..100 ramp in ten samples: the 5th and 95th percentiles are 14.5 and 95.5, so
    // the first sample clamps to the bottom of the ramp and the last to the top. Pinning those two
    // against OpenCV's own LUT catches a wrong range, a wrong denominator or a forgotten clamp.
    sv::DisparityImage disparity;
    disparity.width = 10;
    disparity.height = 1;
    for (int i = 1; i <= disparity.width; ++i) {
        disparity.data.push_back(static_cast<float>(i) * 10.0F);
    }

    const fs::path path = temporary_path("disparity_ramp.png");
    std::string error;
    ASSERT_TRUE(sv::disparity_save(path.string(), disparity, &error, true)) << error;

    const cv::Mat loaded = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    ASSERT_FALSE(loaded.empty());
    ASSERT_EQ(loaded.type(), CV_8UC3);

    cv::Mat levels(1, sv::kColorLevels, CV_8UC1);
    for (int i = 0; i < sv::kColorLevels; ++i) {
        levels.at<std::uint8_t>(0, i) = static_cast<std::uint8_t>(i);
    }
    cv::Mat ramp;
    cv::applyColorMap(levels, ramp, cv::COLORMAP_TURBO);

    // turbo is not monotonic per channel (red falls from purple to cyan before rising to red), so
    // each pixel is mapped back to its level through the LUT instead of comparing components.
    const auto level_of = [&ramp](const cv::Vec3b& pixel) {
        for (int i = 0; i < sv::kColorLevels; ++i) {
            if (ramp.at<cv::Vec3b>(0, i) == pixel) {
                return i;
            }
        }
        return -1;
    };

    EXPECT_EQ(level_of(loaded.at<cv::Vec3b>(0, 0)), 0);
    EXPECT_EQ(level_of(loaded.at<cv::Vec3b>(0, disparity.width - 1)), sv::kColorLevels - 1);
    for (int x = 1; x < disparity.width; ++x) {
        EXPECT_GE(level_of(loaded.at<cv::Vec3b>(0, x)), level_of(loaded.at<cv::Vec3b>(0, x - 1)))
            << "sample " << x << " went backwards along the ramp";
    }
    fs::remove(path);
}

TEST(ImageIo, ColorizedFlatMapStaysVisible) {
    // Every sample equal means the percentiles collapse; the map still has to read as one colour,
    // not as black and not as a division by zero.
    sv::DisparityImage disparity;
    disparity.width = 2;
    disparity.height = 1;
    disparity.data = {5.0F, 5.0F};

    const fs::path path = temporary_path("disparity_flat.png");
    std::string error;
    ASSERT_TRUE(sv::disparity_save(path.string(), disparity, &error, true)) << error;

    const cv::Mat loaded = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    ASSERT_FALSE(loaded.empty());
    EXPECT_EQ(loaded.at<cv::Vec3b>(0, 0), loaded.at<cv::Vec3b>(0, 1));
    EXPECT_NE(loaded.at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 0));
    fs::remove(path);
}

TEST(ImageIo, MissingDirectoryReportsError) {
    const sv::Image image = make_image(2, 2, 1, 1);
    std::string error;
    EXPECT_FALSE(sv::image_save_png("/nonexistent-directory-for-test/frame.png", image, &error));
    EXPECT_NE(error.find("does not exist"), std::string::npos) << error;
    EXPECT_FALSE(fs::exists("/nonexistent-directory-for-test/frame.png"));
}
