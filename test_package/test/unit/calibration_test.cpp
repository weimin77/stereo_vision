#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include "sv_calibration.hpp"

namespace {

namespace fs = std::filesystem;

std::string resource(const std::string& name) { return std::string(RESOURCES_PATH) + "/" + name; }

/// A one-camera override file, written beside the build so the test does not depend on the scene's
/// own intrinsics differing from the stereo file. Fixed notation keeps every value recognisable as
/// a double; the loader parses this with cv::FileStorage, which rejects a malformed list.
fs::path write_override(const std::string& name, const std::string& key, double fx, double cx,
                        double distortion) {
    const auto number = [](double value) {
        std::ostringstream text;
        text << std::fixed << std::setprecision(6) << value;
        return text.str();
    };
    const fs::path path = fs::temp_directory_path() / ("sv_calibration_" + name + ".yml");
    std::ofstream out(path);
    out << "%YAML:1.0\n---\n"
        << key << ": !!opencv-matrix\n"
        << "   rows: 3\n   cols: 3\n   dt: d\n"
        << "   data: [ " << number(fx) << ", 0., " << number(cx) << ", 0., " << number(fx)
        << ", 240., 0., 0., 1. ]\n"
        << (key == "M1" ? "D1" : "D2") << ": !!opencv-matrix\n"
        << "   rows: 1\n   cols: 5\n   dt: d\n"
        << "   data: [ " << number(distortion) << ", 0., 0., 0., 0. ]\n";
    return path;
}

}  // namespace

TEST(Calibration, LoadsStereoFile) {
    sv::Calibration calibration;
    std::string error;
    ASSERT_TRUE(sv::calibration_load_stereo(resource("stereo.yml"), calibration, &error)) << error;

    EXPECT_TRUE(calibration.has_extrinsics);
    EXPECT_FALSE(calibration.has_rectification);
    // The shipped scene: 140 px focal length, 60 mm baseline, no distortion.
    EXPECT_NEAR(calibration.m1[0], 140.0, 1e-9);
    EXPECT_NEAR(calibration.m2[4], 140.0, 1e-9);
    EXPECT_NEAR(calibration.m1[2], 319.5, 1e-9);
    EXPECT_NEAR(calibration.t[0], -0.06, 1e-9);
    ASSERT_EQ(calibration.d1.size(), 5U);
}

TEST(Calibration, MonoFilesOverrideTheStereoIntrinsics) {
    // The scene's mono files carry the stereo file's own intrinsics, so this test writes its own
    // pair: the override mechanism must win, and only for the camera the file belongs to.
    const fs::path left = write_override("left", "M1", 520.0, 318.0, 0.0);
    const fs::path right = write_override("right", "M2", 515.0, 322.0, -0.01);

    sv::Calibration calibration;
    std::string error;
    ASSERT_TRUE(sv::calibration_load_stereo(resource("stereo.yml"), calibration, &error)) << error;
    ASSERT_TRUE(sv::calibration_override_left(left.string(), calibration, &error)) << error;
    ASSERT_TRUE(sv::calibration_override_right(right.string(), calibration, &error)) << error;

    EXPECT_NEAR(calibration.m1[0], 520.0, 1e-9);
    EXPECT_NEAR(calibration.m1[2], 318.0, 1e-9);
    EXPECT_NEAR(calibration.m2[0], 515.0, 1e-9);
    EXPECT_NEAR(calibration.m2[2], 322.0, 1e-9);
    EXPECT_NEAR(calibration.d2[0], -0.01, 1e-9);
    // The extrinsics still come from the stereo file.
    EXPECT_TRUE(calibration.has_extrinsics);
    EXPECT_NEAR(calibration.t[0], -0.06, 1e-9);

    fs::remove(left);
    fs::remove(right);
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
