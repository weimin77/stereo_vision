#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "sv_depth.hpp"
#include "sv_image_io.hpp"
#include "sv_pointcloud.hpp"
#include "sv_sgbm.hpp"
#include "synthetic.hpp"

namespace {

namespace fs = std::filesystem;

constexpr int kKnownShift = 20;
constexpr int kNumDisparities = 64;
constexpr double kDisparityTolerance = 0.5;
constexpr double kDepthTolerance = 0.02;
constexpr double kCoordinateTolerance = 1e-4;
constexpr double kFocal = synthetic::kTestFocal;
constexpr double kBaseline = synthetic::kTestBaseline;

fs::path temporary_path(const std::string& name) {
    const fs::path dir = fs::temp_directory_path() / "stereo_vision_cloud_tests";
    fs::create_directories(dir);
    return dir / name;
}

sv::RectifyGeometry identity_geometry(int width, int height) {
    sv::RectifyGeometry geometry;
    geometry.width = width;
    geometry.height = height;
    geometry.fx = kFocal;
    geometry.fy = kFocal;
    geometry.cx = static_cast<double>(width) / 2.0;
    geometry.cy = static_cast<double>(height) / 2.0;
    geometry.baseline = kBaseline;
    geometry.focal_baseline = kFocal * kBaseline;
    return geometry;
}

sv::SgbmParams known_shift_params() {
    sv::SgbmParams params;
    params.min_disparity = 0;
    params.num_disparities = kNumDisparities;
    params.block_size = 5;
    params.mode = sv::SgbmMode::Sgbm;
    return params;
}

}  // namespace

TEST(Sgbm, RecoversAKnownHorizontalShift) {
    const sv::Image left = synthetic::blocky_texture(synthetic::kTestWidth, synthetic::kTestHeight, 11U);
    const sv::Image right = synthetic::shift_horizontal(left, kKnownShift);

    sv::DisparityImage disparity;
    std::string error;
    ASSERT_TRUE(sv::sgbm_compute(left, right, {}, known_shift_params(), disparity, &error)) << error;

    const double median = synthetic::median_of_finite(disparity.data);
    EXPECT_NEAR(median, static_cast<double>(kKnownShift), kDisparityTolerance) << "median disparity " << median;
}

TEST(Sgbm, InvalidMaskSuppressesPixels) {
    const sv::Image left = synthetic::blocky_texture(320, 240, 13U);
    const sv::Image right = synthetic::shift_horizontal(left, kKnownShift);

    std::vector<std::uint8_t> mask(static_cast<std::size_t>(left.width) * left.height, sv::kChannelMax);
    for (int y = 0; y < left.height; ++y) {
        for (int x = 0; x < left.width / 2; ++x) {
            mask[(static_cast<std::size_t>(y) * left.width) + x] = 0;
        }
    }

    sv::DisparityImage disparity;
    std::string error;
    ASSERT_TRUE(sv::sgbm_compute(left, right, mask, known_shift_params(), disparity, &error)) << error;

    for (std::size_t i = 0; i < disparity.data.size(); ++i) {
        if (mask[i] == 0) {
            EXPECT_FALSE(std::isfinite(disparity.data[i])) << "pixel " << i;
        }
    }
}

TEST(Sgbm, RejectsMalformedParameters) {
    std::string error;
    sv::SgbmParams params = known_shift_params();

    params.num_disparities = 40;
    EXPECT_FALSE(sv::sgbm_validate(params, &error));
    EXPECT_NE(error.find("multiple of 16"), std::string::npos) << error;

    params = known_shift_params();
    params.block_size = 4;
    EXPECT_FALSE(sv::sgbm_validate(params, &error));

    params = known_shift_params();
    params.p2_factor = params.p1_factor;
    EXPECT_FALSE(sv::sgbm_validate(params, &error));

    params = known_shift_params();
    params.min_disparity = -1;
    EXPECT_FALSE(sv::sgbm_validate(params, &error));
}

TEST(Sgbm, RejectsMismatchedImageSizes) {
    const sv::Image left = synthetic::blocky_texture(320, 240, 17U);
    const sv::Image right = synthetic::blocky_texture(160, 120, 17U);
    sv::DisparityImage disparity;
    std::string error;
    EXPECT_FALSE(sv::sgbm_compute(left, right, {}, known_shift_params(), disparity, &error));
    EXPECT_FALSE(error.empty());
}

TEST(Depth, ConvertsDisparityToMetres) {
    sv::DisparityImage disparity;
    disparity.width = 3;
    disparity.height = 1;
    disparity.data = {20.0F, 0.0F, std::numeric_limits<float>::quiet_NaN()};

    const sv::RectifyGeometry geometry = identity_geometry(3, 1);
    sv::DepthImage depth;
    std::string error;
    ASSERT_TRUE(sv::depth_from_disparity(disparity, geometry, 0.0, 0.0, depth, &error)) << error;

    EXPECT_NEAR(depth.data[0], (kFocal * kBaseline) / 20.0, 1e-6);
    EXPECT_FALSE(std::isfinite(depth.data[1]));
    EXPECT_FALSE(std::isfinite(depth.data[2]));
}

TEST(Depth, HonoursThePrincipalPointOffset) {
    sv::DisparityImage disparity;
    disparity.width = 1;
    disparity.height = 1;
    disparity.data = {20.0F};

    sv::RectifyGeometry geometry = identity_geometry(1, 1);
    geometry.doffs = 2.0;
    sv::DepthImage depth;
    std::string error;
    ASSERT_TRUE(sv::depth_from_disparity(disparity, geometry, 0.0, 0.0, depth, &error)) << error;
    EXPECT_NEAR(depth.data[0], (kFocal * kBaseline) / (20.0 - 2.0), 1e-6);
}

TEST(Depth, ClampRejectsOutOfRangeDepths) {
    sv::DisparityImage disparity;
    disparity.width = 2;
    disparity.height = 1;
    disparity.data = {20.0F, 2.0F};

    const sv::RectifyGeometry geometry = identity_geometry(2, 1);
    sv::DepthImage depth;
    std::string error;
    ASSERT_TRUE(sv::depth_from_disparity(disparity, geometry, 1.0, 3.0, depth, &error)) << error;
    EXPECT_NEAR(depth.data[0], 2.5, 1e-6);
    EXPECT_FALSE(std::isfinite(depth.data[1]));
}

TEST(Sgbm, DoesNotRejectPixelsForALeftRightMismatch) {
    // The matcher runs once, so there is no right-to-left map to compare against. A pixel that
    // the old consistency check would have thrown away must now survive: an occluded bar, where
    // the left view sees a surface the right view does not.
    const sv::Image left = synthetic::blocky_texture(160, 120, 29U);
    sv::Image right = synthetic::shift_horizontal(left, kKnownShift);
    // Punch a hole in the right view so no left pixel there has a consistent partner.
    for (int y = 40; y < 80; ++y) {
        for (int x = 40; x < 120; ++x) {
            right.data[(static_cast<std::size_t>(y) * right.width) + x] = 0;
        }
    }

    sv::DisparityImage disparity;
    std::string error;
    ASSERT_TRUE(sv::sgbm_compute(left, right, {}, known_shift_params(), disparity, &error)) << error;

    std::size_t hole_finite = 0;
    for (int y = 40; y < 80; ++y) {
        for (int x = 40; x < 120; ++x) {
            if (std::isfinite(disparity.data[(static_cast<std::size_t>(y) * disparity.width) + x])) {
                ++hole_finite;
            }
        }
    }
    // Measured on this input: 1757/3200 with the single pass, 1017/3200 when the second pass
    // still ran and its consistency check discarded the mismatched half. The threshold sits
    // between the two, so re-adding the check turns this test red instead of quietly costing
    // another 90 ms per frame.
    constexpr std::size_t kHolePixels = 40U * 80U;
    EXPECT_GT(hole_finite, kHolePixels * 42U / 100U) << hole_finite << " of " << kHolePixels;
}

TEST(PointCloud, BackProjectsDepthWithThePinholeModel) {
    const sv::RectifyGeometry geometry = identity_geometry(4, 2);
    sv::DepthImage depth;
    depth.width = 4;
    depth.height = 2;
    depth.data = {2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F, 2.0F};

    sv::PointCloud cloud;
    std::string error;
    ASSERT_TRUE(sv::pointcloud_from_depth(depth, geometry, false, cloud, &error)) << error;
    ASSERT_EQ(cloud.points.size(), depth.data.size());

    const std::size_t index = 5;  // x = 1, y = 1
    const double expected_x = (1.0 - geometry.cx) * 2.0 / geometry.fx;
    const double expected_y = (1.0 - geometry.cy) * 2.0 / geometry.fy;
    EXPECT_NEAR(cloud.points[index].x, expected_x, kCoordinateTolerance);
    EXPECT_NEAR(cloud.points[index].y, expected_y, kCoordinateTolerance);
    EXPECT_NEAR(cloud.points[index].z, 2.0, kCoordinateTolerance);
}

TEST(PointCloud, KeepInvalidProducesOneEntryPerPixel) {
    const sv::RectifyGeometry geometry = identity_geometry(2, 1);
    sv::DepthImage depth;
    depth.width = 2;
    depth.height = 1;
    depth.data = {2.0F, std::numeric_limits<float>::quiet_NaN()};

    sv::PointCloud valid_only;
    sv::PointCloud kept;
    std::string error;
    ASSERT_TRUE(sv::pointcloud_from_depth(depth, geometry, false, valid_only, &error)) << error;
    ASSERT_TRUE(sv::pointcloud_from_depth(depth, geometry, true, kept, &error)) << error;
    EXPECT_EQ(valid_only.points.size(), 1U);
    EXPECT_EQ(kept.points.size(), 2U);
    EXPECT_FALSE(std::isfinite(kept.points[1].z));
}

TEST(PointCloud, CloudCarriesGeometryOnly) {
    // The cloud has three fields per point. A test that only counts them would still pass if a
    // fourth were appended, so the size of Point itself is the assertion.
    EXPECT_EQ(sizeof(sv::Point), 3U * sizeof(float));

    const sv::RectifyGeometry geometry = identity_geometry(2, 1);
    sv::DepthImage depth;
    depth.width = 2;
    depth.height = 1;
    depth.data = {2.0F, 2.0F};

    sv::PointCloud cloud;
    std::string error;
    ASSERT_TRUE(sv::pointcloud_from_depth(depth, geometry, false, cloud, &error)) << error;
    ASSERT_EQ(cloud.points.size(), 2U);
    EXPECT_NEAR(cloud.points[1].z, 2.0, kCoordinateTolerance);
}

TEST(PointCloud, PcdAsciiHeaderAndCount) {
    sv::PointCloud cloud;
    cloud.points.resize(2);
    cloud.points[0].x = 1.0F;
    cloud.points[0].y = 2.0F;
    cloud.points[0].z = 3.0F;
    cloud.points[1].x = std::numeric_limits<float>::quiet_NaN();

    const fs::path path = temporary_path("cloud_ascii.pcd");
    std::string error;
    ASSERT_TRUE(sv::pointcloud_save_pcd(path.string(), cloud, false, &error)) << error;

    std::ifstream input(path);
    ASSERT_TRUE(input.good());
    std::string line;
    bool saw_points = false;
    bool saw_ascii = false;
    std::size_t data_lines = 0;
    bool in_data = false;
    while (std::getline(input, line)) {
        if (line.rfind("POINTS ", 0) == 0) {
            saw_points = line == "POINTS 2";
            continue;
        }
        if (line.rfind("DATA ", 0) == 0) {
            saw_ascii = line == "DATA ascii";
            in_data = true;
            continue;
        }
        if (!in_data) {
            continue;
        }
        ++data_lines;
        // Three columns: a fourth would be the rgb field this writer no longer emits. Split on
        // spaces rather than stream into doubles, because "nan" is a valid column and a numeric
        // stream rejects it.
        std::size_t columns = 0;
        bool in_token = false;
        for (const char c : line) {
            const bool separator = c == ' ';
            if (!separator && !in_token) {
                ++columns;
            }
            in_token = !separator;
        }
        EXPECT_EQ(columns, 3U) << "row " << data_lines << " is '" << line << "'";
    }
    EXPECT_TRUE(saw_points);
    EXPECT_TRUE(saw_ascii);
    EXPECT_EQ(data_lines, 2U);
    fs::remove(path);
}

TEST(PointCloud, PcdBinarySizeMatchesThePointCount) {
    sv::PointCloud cloud;
    cloud.points.resize(3);
    for (auto& point : cloud.points) {
        point.x = 0.1F;
        point.z = 2.0F;
    }

    const fs::path path = temporary_path("cloud_binary.pcd");
    std::string error;
    ASSERT_TRUE(sv::pointcloud_save_pcd(path.string(), cloud, true, &error)) << error;

    std::ifstream input(path, std::ios::binary);
    ASSERT_TRUE(input.good());
    std::string line;
    std::size_t header_bytes = 0;
    bool saw_binary = false;
    while (std::getline(input, line)) {
        header_bytes += line.size() + 1;
        if (line.rfind("DATA ", 0) == 0) {
            saw_binary = line == "DATA binary";
            break;
        }
    }
    EXPECT_TRUE(saw_binary);
    constexpr std::size_t kBytesPerPoint = 3U * sizeof(float);
    EXPECT_EQ(fs::file_size(path), header_bytes + (3U * kBytesPerPoint));
    fs::remove(path);
}

TEST(PointCloud, PlyHeaderDeclaresExactlyThreeProperties) {
    sv::PointCloud cloud;
    cloud.points.resize(1);
    cloud.points[0].x = 1.0F;

    const fs::path path = temporary_path("cloud_ascii.ply");
    std::string error;
    ASSERT_TRUE(sv::pointcloud_save_ply(path.string(), cloud, false, &error)) << error;

    std::ifstream input(path);
    ASSERT_TRUE(input.good());
    std::string line;
    bool saw_format = false;
    bool saw_vertex_count = false;
    bool saw_colour = false;
    std::vector<std::string> properties;
    while (std::getline(input, line)) {
        saw_format = saw_format || line == "format ascii 1.0";
        saw_vertex_count = saw_vertex_count || line == "element vertex 1";
        if (line.rfind("property ", 0) == 0) {
            properties.push_back(line);
            saw_colour = saw_colour || line == "property uchar red" || line == "property uchar green" ||
                         line == "property uchar blue";
        }
    }
    EXPECT_TRUE(saw_format);
    EXPECT_TRUE(saw_vertex_count);
    EXPECT_FALSE(saw_colour) << "the cloud is geometry only, so no colour property may be declared";
    ASSERT_EQ(properties.size(), 3U);
    EXPECT_EQ(properties[0], "property float x");
    EXPECT_EQ(properties[1], "property float y");
    EXPECT_EQ(properties[2], "property float z");
    fs::remove(path);
}

TEST(PointCloud, PcdHeaderDeclaresExactlyThreeFields) {
    sv::PointCloud cloud;
    cloud.points.resize(1);
    cloud.points[0].z = 1.0F;

    const fs::path path = temporary_path("cloud_fields.pcd");
    std::string error;
    ASSERT_TRUE(sv::pointcloud_save_pcd(path.string(), cloud, false, &error)) << error;

    std::ifstream input(path);
    ASSERT_TRUE(input.good());
    std::string line;
    bool saw_fields = false;
    bool saw_size = false;
    bool saw_type = false;
    bool saw_count = false;
    while (std::getline(input, line)) {
        saw_fields = saw_fields || line == "FIELDS x y z";
        saw_size = saw_size || line == "SIZE 4 4 4";
        saw_type = saw_type || line == "TYPE F F F";
        saw_count = saw_count || line == "COUNT 1 1 1";
    }
    EXPECT_TRUE(saw_fields) << "a fourth field would break the offset every PCD reader uses";
    EXPECT_TRUE(saw_size);
    EXPECT_TRUE(saw_type);
    EXPECT_TRUE(saw_count);
    fs::remove(path);
}

TEST(PointCloud, SaveRejectsAnUnknownExtension) {
    sv::PointCloud cloud;
    std::string error;
    EXPECT_FALSE(sv::pointcloud_save(temporary_path("cloud.xyz").string(), cloud, false, &error));
    EXPECT_NE(error.find(".pcd"), std::string::npos) << error;
}

TEST(PointCloud, SaveRejectsAMissingDirectory) {
    sv::PointCloud cloud;
    std::string error;
    EXPECT_FALSE(sv::pointcloud_save("/nonexistent-directory-for-test/cloud.pcd", cloud, false, &error));
    EXPECT_FALSE(fs::exists("/nonexistent-directory-for-test/cloud.pcd"));
}
