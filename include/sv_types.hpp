// Conan::ImportStart
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
// Conan::ImportEnd

namespace sv {

/// Largest value an 8-bit channel can hold; also the "valid" value of a mask.
inline constexpr std::uint8_t kChannelMax = 255;

/// Number of elements in a 4x4 matrix stored row-major.
inline constexpr std::size_t kMatrix4Size = 16;

/// Number of elements in a 3x3 matrix stored row-major.
inline constexpr std::size_t kMatrix3Size = 9;

/**
 * @brief [en] 8-bit image, 1 (gray) or 3 (BGR) interleaved channels
 * @brief [zh] 8 位图像，1（灰度）或 3（BGR）通道，按行交错存储
 * @since 1.0
 * @exporter
 */
struct Image {
    int width = 0;
    int height = 0;
    int channels = 0;
    std::vector<std::uint8_t> data;
};

/**
 * @brief [en] Sub-pixel disparity map in pixels; NaN marks a pixel with no valid match
 * @brief [zh] 亚像素视差图，单位像素；NaN 表示该像素没有有效匹配
 * @since 1.0
 * @exporter
 */
struct DisparityImage {
    int width = 0;
    int height = 0;
    std::vector<float> data;
};

/**
 * @brief [en] Depth map in metres; NaN marks a pixel with no valid depth
 * @brief [zh] 深度图，单位米；NaN 表示该像素没有有效深度
 * @since 1.0
 * @exporter
 */
struct DepthImage {
    int width = 0;
    int height = 0;
    std::vector<float> data;
};

/**
 * @brief [en] One back-projected point
 * @brief [zh] 一个反投影得到的点
 * @since 1.0
 * @exporter
 */
struct Point {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

/**
 * @brief [en] Unorganised point cloud in the rectified left camera frame
 * @brief [zh] 非结构化点云，坐标系为校正后的左相机坐标系
 *
 * Geometry only: the cloud carries x, y, z and nothing else, so a PCD or PLY written from it has
 * three fields. Colour lives in the images the run already produced, and mixing the two would make
 * the cloud's size depend on whether an RGB pair happened to be available.
 *
 * @since 1.0
 * @exporter
 */
struct PointCloud {
    std::vector<Point> points;
};

/**
 * @brief [en] Rectified stereo pair plus the geometry a matcher needs
 * @brief [zh] 校正后的双目图像对及匹配所需的几何参数
 * @since 1.0
 * @exporter
 */
struct RectifiedPair {
    Image left;
    Image right;
    std::vector<std::uint8_t> valid_mask;
};

/**
 * @brief [en] Geometry of the rectified views, all lengths in pixels unless stated
 * @brief [zh] 校正后视图的几何参数，除注明外长度单位为像素
 * @since 1.0
 * @exporter
 */
struct RectifyGeometry {
    int width = 0;
    int height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    /// Horizontal principal point offset between the rectified views.
    double doffs = 0.0;
    /// Metric baseline in metres.
    double baseline = 0.0;
    /// fx * |baseline|, the numerator of the depth equation.
    double focal_baseline = 0.0;
    std::array<double, kMatrix4Size> q{};
};

/**
 * @brief [en] Timings and statistics of one pipeline run
 * @brief [zh] 一次流水线运行的耗时与统计
 * @since 1.0
 * @exporter
 */
struct RunReport {
    double rectify_ms = 0.0;
    double match_ms = 0.0;
    double depth_ms = 0.0;
    int rectified_width = 0;
    int rectified_height = 0;
    int min_disparity = 0;
    int num_disparities = 0;
    double valid_ratio = 0.0;
    double disparity_p05 = 0.0;
    double disparity_p50 = 0.0;
    double disparity_p95 = 0.0;
    double depth_p05 = 0.0;
    double depth_p50 = 0.0;
    double depth_p95 = 0.0;
    std::string output_directory_resolved;
};

/**
 * @brief [en] Whether an image carries a consistent size and pixel buffer
 * @brief [zh] 判断图像尺寸与像素缓冲区是否自洽
 * @param image image to check / 待检查图像
 * @return true when width, height, channels and buffer size agree
 * @since 1.0
 * @exporter
 */
bool image_valid(const Image& image);

/**
 * @brief [en] Whether a disparity map carries a consistent size and buffer
 * @brief [zh] 判断视差图尺寸与缓冲区是否自洽
 * @param disparity map to check / 待检查视差图
 * @return true when width, height and buffer size agree
 * @since 1.0
 * @exporter
 */
bool disparity_valid(const DisparityImage& disparity);

/**
 * @brief [en] Whether a depth map carries a consistent size and buffer
 * @brief [zh] 判断深度图尺寸与缓冲区是否自洽
 * @param depth map to check / 待检查深度图
 * @return true when width, height and buffer size agree
 * @since 1.0
 * @exporter
 */
bool depth_valid(const DepthImage& depth);

/**
 * @brief [en] Convert a BGR image to gray, leaving a gray image untouched
 * @brief [zh] 把 BGR 图像转为灰度，灰度图像保持不变
 * @param source input image / 输入图像
 * @param gray output gray image / 输出灰度图像
 * @since 1.0
 * @exporter
 */
void image_to_gray(const Image& source, Image& gray);

/**
 * @brief [en] The 5th, 50th and 95th percentiles of the finite entries of a buffer
 * @brief [zh] 缓冲区中有限值部分的 5%、50%、95% 分位数
 * @param values input samples / 输入样本
 * @param out receives p05, p50 and p95; all zero when no finite sample exists
 * @since 1.0
 * @exporter
 */
void percentiles(const std::vector<float>& values, double (&out)[3]); // NOLINT(cppcoreguidelines-avoid-c-arrays)

} // namespace sv
