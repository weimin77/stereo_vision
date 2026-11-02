#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "sv_rectify.hpp"
#include "synthetic.hpp"

namespace {

constexpr double kGeometryTolerance = 1e-6;
constexpr std::size_t kMatrixDim = 4;
constexpr double kIdentityTolerance = 1e-9;

/// Apply the 4x4 Q matrix to (u, v, disparity, 1) and return the 3D point.
void reproject(const std::array<double, sv::kMatrix4Size>& q, double u, double v, double disparity, double (&out)[3]) {
    const double vector[kMatrixDim] = {u, v, disparity, 1.0};
    double homogeneous[kMatrixDim] = {0.0, 0.0, 0.0, 0.0};
    for (std::size_t row = 0; row < kMatrixDim; ++row) {
        for (std::size_t column = 0; column < kMatrixDim; ++column) {
            homogeneous[row] += q[(row * kMatrixDim) + column] * vector[column];
        }
    }
    out[0] = homogeneous[0] / homogeneous[3];
    out[1] = homogeneous[1] / homogeneous[3];
    out[2] = homogeneous[2] / homogeneous[3];
}

}  // namespace

TEST(Rectify, GeometryOfAnAlignedRig) {
    sv::Rectifier rectifier;
    sv::RectifyOptions options;
    options.alpha = 0.0;
    options.crop_to_common_roi = false;

    std::string error;
    ASSERT_TRUE(rectifier.prepare(synthetic::pinhole_rig(), synthetic::kTestWidth, synthetic::kTestHeight, options,
                                  &error))
        << error;
    const sv::RectifyGeometry& geometry = rectifier.geometry();
    EXPECT_NEAR(geometry.fx, synthetic::kTestFocal, kGeometryTolerance);
    EXPECT_NEAR(geometry.baseline, synthetic::kTestBaseline, kGeometryTolerance);
    EXPECT_NEAR(geometry.doffs, 0.0, kGeometryTolerance);
    EXPECT_NEAR(geometry.focal_baseline, synthetic::kTestFocal * synthetic::kTestBaseline, kGeometryTolerance);
    EXPECT_EQ(geometry.width, synthetic::kTestWidth);
    EXPECT_EQ(geometry.height, synthetic::kTestHeight);
}

TEST(Rectify, IdentityMapsPreserveTheInputPixels) {
    const sv::Image texture = synthetic::blocky_texture(synthetic::kTestWidth, synthetic::kTestHeight, 1U);
    const sv::Image shifted = synthetic::shift_horizontal(texture, 20);

    sv::Rectifier rectifier;
    sv::RectifyOptions options;
    options.crop_to_common_roi = false;
    std::string error;
    ASSERT_TRUE(rectifier.prepare(synthetic::pinhole_rig(), texture.width, texture.height, options, &error)) << error;

    sv::RectifiedPair pair;
    ASSERT_TRUE(rectifier.apply(texture, shifted, pair, &error)) << error;
    ASSERT_EQ(pair.left.data.size(), texture.data.size());
    std::size_t differing = 0;
    for (std::size_t i = 0; i < texture.data.size(); ++i) {
        if (pair.left.data[i] != texture.data[i]) {
            ++differing;
        }
    }
    EXPECT_EQ(differing, 0U);
    ASSERT_EQ(pair.valid_mask.size(), texture.data.size());
    EXPECT_EQ(pair.valid_mask[0], sv::kChannelMax);
}

TEST(Rectify, QReprojectionAgreesWithTheDepthEquation) {
    sv::Rectifier rectifier;
    sv::RectifyOptions options;
    options.crop_to_common_roi = true;
    std::string error;
    ASSERT_TRUE(rectifier.prepare(synthetic::pinhole_rig(), synthetic::kTestWidth, synthetic::kTestHeight, options,
                                  &error))
        << error;

    const sv::RectifyGeometry& geometry = rectifier.geometry();
    const double disparity = 20.0;
    const double expected_z = geometry.focal_baseline / (disparity - geometry.doffs);
    const double u = 300.0;
    const double v = 200.0;

    double point[3] = {0.0, 0.0, 0.0};
    reproject(geometry.q, u, v, disparity, point);
    EXPECT_NEAR(point[2], expected_z, kGeometryTolerance * expected_z);
    EXPECT_NEAR(point[0], (u - geometry.cx) * expected_z / geometry.fx, kGeometryTolerance * expected_z);
    EXPECT_NEAR(point[1], (v - geometry.cy) * expected_z / geometry.fy, kGeometryTolerance * expected_z);
}

TEST(Rectify, RejectsACalibrationWithTheRightCameraOnThePositiveXSide) {
    sv::Rectifier rectifier;
    sv::RectifyOptions options;
    std::string error;
    EXPECT_FALSE(rectifier.prepare(synthetic::mirrored_rig(), synthetic::kTestWidth, synthetic::kTestHeight, options,
                                   &error));
    EXPECT_NE(error.find("+x"), std::string::npos) << error;
}

TEST(Rectify, RejectsAnOutputSizeWithAStoredRectification) {
    sv::Calibration calibration = synthetic::pinhole_rig();
    calibration.has_extrinsics = false;
    calibration.has_rectification = true;
    calibration.r1 = calibration.r;
    calibration.r2 = calibration.r;
    calibration.p1[0] = synthetic::kTestFocal;
    calibration.p1[2] = synthetic::kTestCx;
    calibration.p1[5] = synthetic::kTestFocal;
    calibration.p1[6] = synthetic::kTestCy;
    calibration.p1[10] = 1.0;
    calibration.p2 = calibration.p1;
    calibration.p2[3] = -synthetic::kTestFocal * synthetic::kTestBaseline;

    sv::Rectifier rectifier;
    sv::RectifyOptions options;
    options.new_width = 320;
    options.new_height = 240;
    std::string error;
    EXPECT_FALSE(rectifier.prepare(calibration, synthetic::kTestWidth, synthetic::kTestHeight, options, &error));
    EXPECT_NE(error.find("new_size"), std::string::npos) << error;
}

TEST(Rectify, DisabledRectificationCopiesTheInputs) {
    const sv::Image texture = synthetic::blocky_texture(64, 48, 7U);
    sv::Rectifier rectifier;
    sv::RectifyOptions options;
    options.enabled = false;

    std::string error;
    ASSERT_TRUE(rectifier.prepare(synthetic::pinhole_rig(), texture.width, texture.height, options, &error)) << error;
    sv::RectifiedPair pair;
    ASSERT_TRUE(rectifier.apply(texture, texture, pair, &error)) << error;
    EXPECT_EQ(pair.left.data, texture.data);
    EXPECT_EQ(pair.right.data, texture.data);
    EXPECT_EQ(pair.valid_mask.size(), texture.data.size());
    EXPECT_NEAR(rectifier.geometry().doffs, 0.0, kIdentityTolerance);
}

TEST(Rectify, ApplyRejectsAnUnpreparedRectifier) {
    const sv::Image texture = synthetic::blocky_texture(16, 16, 3U);
    sv::Rectifier rectifier;
    sv::RectifiedPair pair;
    std::string error;
    EXPECT_FALSE(rectifier.apply(texture, texture, pair, &error));
    EXPECT_FALSE(error.empty());
}

TEST(Rectify, ApplyRejectsImagesOfTheWrongSize) {
    const sv::Image small = synthetic::blocky_texture(32, 24, 5U);
    sv::Rectifier rectifier;
    sv::RectifyOptions options;
    std::string error;
    ASSERT_TRUE(rectifier.prepare(synthetic::pinhole_rig(), synthetic::kTestWidth, synthetic::kTestHeight, options,
                                  &error))
        << error;
    sv::RectifiedPair pair;
    EXPECT_FALSE(rectifier.apply(small, small, pair, &error));
    EXPECT_FALSE(error.empty());
}
