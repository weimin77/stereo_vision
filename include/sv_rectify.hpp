// Conan::ImportStart
#pragma once

#include <memory>
#include <string>

#include "sv_calibration.hpp"
#include "sv_types.hpp"
// Conan::ImportEnd

namespace sv {

/**
 * @brief [en] Knobs of the epipolar rectification step
 * @brief [zh] 极线校正步骤的参数
 * @since 1.0
 * @exporter
 */
struct RectifyOptions {
    /// When false the inputs are assumed to be already rectified and only the mask is synthesised.
    bool enabled = true;
    /// Free scaling in [0, 1]; 0 keeps every source pixel, 1 keeps only the common valid region.
    double alpha = 0.0;
    /// Output size before cropping; zero means "same as the input".
    int new_width = 0;
    int new_height = 0;
    /// Crop to the intersection of the two valid regions, so both views share one rectangle.
    bool crop_to_common_roi = true;
    /// Fixed-point maps are smaller and faster; float maps avoid the 1/32 px quantisation.
    bool fixed_point_maps = true;
};

/**
 * @brief [en] Epipolar rectifier: stereoRectify, undistort maps, common ROI and a validity mask
 * @brief [zh] 极线校正器：stereoRectify、去畸变映射、公共有效区与有效性掩膜
 *
 * The class owns the OpenCV map objects, so no OpenCV type appears in this header.
 *
 * @since 1.0
 * @exporter
 */
class Rectifier {
  public:
    Rectifier();
    ~Rectifier();
    Rectifier(const Rectifier&) = delete;
    Rectifier& operator=(const Rectifier&) = delete;
    Rectifier(Rectifier&& other) noexcept;
    Rectifier& operator=(Rectifier&& other) noexcept;

    /**
     * @brief [en] Compute the rectification for one image size
     * @brief [zh] 针对一个图像尺寸计算校正变换
     * @param calibration stereo calibration / 双目标定
     * @param width input image width / 输入图像宽度
     * @param height input image height / 输入图像高度
     * @param options rectification options / 校正参数
     * @param error optional message, written on failure / 失败时写入原因
     * @return true on success
     * @since 1.0
     * @exporter
     */
    bool prepare(const Calibration& calibration, int width, int height, const RectifyOptions& options,
                 std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

    /**
     * @brief [en] Rectify one stereo pair with the prepared maps
     * @brief [zh] 用已准备的映射校正一对图像
     * @param left left image / 左图
     * @param right right image / 右图
     * @param pair receives the rectified images and mask / 接收校正图与掩膜
     * @param error optional message, written on failure / 失败时写入原因
     * @return true on success
     * @since 1.0
     * @exporter
     */
    bool apply(const Image& left, const Image& right, RectifiedPair& pair,
               std::string* error) const; // NOLINT(bugprone-easily-swappable-parameters)

    /**
     * @brief [en] Geometry of the rectified views
     * @brief [zh] 校正后视图的几何参数
     * @return the geometry prepared by prepare(); zeroed before that
     * @since 1.0
     * @exporter
     */
    const RectifyGeometry& geometry() const;

    /**
     * @brief [en] Whether prepare() succeeded
     * @brief [zh] prepare() 是否已成功
     * @return true when the maps are ready
     * @since 1.0
     * @exporter
     */
    bool prepared() const;

    /**
     * @brief [en] Drop the prepared maps
     * @brief [zh] 丢弃已准备的映射
     * @since 1.0
     * @exporter
     */
    void reset();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sv
