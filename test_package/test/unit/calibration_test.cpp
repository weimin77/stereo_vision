#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "sv_calibration.hpp"

namespace {

namespace fs = std::filesystem;

std::string resource(const std::string& name) { return std::string(RESOURCES_PATH) + "/" + name; }

}  // namespace

TEST(Calibration, LoadsStereoFile) {
    sv::Calibration calibration;
    std::string error;
    ASSERT_TRUE(sv::calibration_load_stereo(resource("stereo.yml"), calibration, &error)) << error;

    EXPECT_TRUE(calibration.has_extrinsics);
    EXPECT_FALSE(calibration.has_rectification);
    EXPECT_NEAR(calibration.m1[0], 500.0, 1e-9);
    EXPECT_NEAR(calibration.m2[4], 500.0, 1e-9);
    EXPECT_NEAR(calibration.t[0], -0.1, 1e-9);
    ASSERT_EQ(calibration.d1.size(), 5U);
}

TEST(Calibration, MonoFilesOverrideTheStereoIntrinsics) {
    sv::Calibration calibration;
    std::string error;
    ASSERT_TRUE(sv::calibration_load_stereo(resource("stereo.yml"), calibration, &error)) << error;
    ASSERT_TRUE(sv::calibration_override_left(resource("left_mono.yml"), calibration, &error)) << error;
    ASSERT_TRUE(sv::calibration_override_right(resource("right_mono.yml"), calibration, &error)) << error;

    EXPECT_NEAR(calibration.m1[0], 520.0, 1e-9);
    EXPECT_NEAR(calibration.m1[2], 318.0, 1e-9);
    EXPECT_NEAR(calibration.m2[0], 515.0, 1e-9);
    EXPECT_NEAR(calibration.d2[0], -0.01, 1e-9);
    // The extrinsics still come from the stereo file.
    EXPECT_TRUE(calibration.has_extrinsics);
    EXPECT_NEAR(calibration.t[0], -0.1, 1e-9);
}

TEST(Calibration, MissingFileIsReported) {
    sv::Calibration calibration;
    std::string error;
    EXPECT_FALSE(sv::calibration_load_stereo(resource("does-not-exist.yml"), calibration, &error));
    EXPECT_FALSE(error.empty());
}

TEST(Calibration, MonoFileWithoutMatchingKeysIsRejected) {
    sv::Calibration calibration;
    std::string error;
    ASSERT_TRUE(sv::calibration_load_stereo(resource("stereo.yml"), calibration, &error)) << error;

    // A right-camera file cannot override the left camera.
    EXPECT_FALSE(sv::calibration_override_left(resource("right_mono.yml"), calibration, &error));
    EXPECT_NE(error.find("M1"), std::string::npos) << error;
}

TEST(Calibration, ValidationRejectsNonPositiveFocalLength) {
    sv::Calibration calibration;
    std::string error;
    ASSERT_TRUE(sv::calibration_load_stereo(resource("stereo.yml"), calibration, &error)) << error;
    EXPECT_TRUE(sv::calibration_valid(calibration, &error)) << error;

    calibration.m1[0] = 0.0;
    EXPECT_FALSE(sv::calibration_valid(calibration, &error));
    EXPECT_NE(error.find("focal length"), std::string::npos) << error;
}
