// Conan::ImportStart
#pragma once

#include <memory>
#include <string>

#include "sv_calibration.hpp"
#include "sv_config.hpp"
#include "sv_depth.hpp"
#include "sv_image_io.hpp"
#include "sv_pointcloud.hpp"
#include "sv_rectify.hpp"
#include "sv_sgbm.hpp"
#include "sv_types.hpp"
// Conan::ImportEnd

namespace sv {

/**
 * @brief [en] One-shot stereo pipeline: rectify, match, convert to depth, then expose results
 * @brief [zh] 一次性双目流水线：校正、匹配、转深度，然后对外提供结果
 *
 * run() only computes; it never creates a directory or writes a file. Results are read with
 * get_* and written with save_*, so the caller decides which artefacts exist. Images are
 * injected through set_input_image and never named by the configuration file.
 *
 * The object is not thread-safe: get_pointcloud() caches on first use, so one instance must
 * not be read from several threads at once.
 *
 * @since 1.0
 * @exporter
 */
class Pipeline {
  public:
    /**
     * @brief [en] Build a pipeline from an already validated configuration
     * @brief [zh] 用已校验的配置构造流水线
     * @param config run configuration / 运行配置
     * @since 1.0
     * @exporter
     */
    explicit Pipeline(const Config& config);
    ~Pipeline();
    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;
    Pipeline(Pipeline&&) noexcept;
    Pipeline& operator=(Pipeline&&) noexcept;

    /**
     * @brief [en] Provide the calibration as values
     * @brief [zh] 直接提供标定数值
     * @param calibration stereo calibration / 双目标定
     * @return true when the calibration is complete enough to rectify
     * @since 1.0
     * @exporter
     */
    bool set_calibration(const Calibration& calibration);

    /**
     * @brief [en] Load the calibration from a stereo file and optional per-camera files
     * @brief [zh] 从双目文件与可选单目文件读取标定
     * @param stereo_yml stereo.yml path / stereo.yml 路径
     * @param left_mono_yml left intrinsics path, may be empty / 左目内参路径，可为空
     * @param right_mono_yml right intrinsics path, may be empty / 右目内参路径，可为空
     * @return true on success; on failure last_error() explains why
     * @since 1.0
     * @exporter
     */
    bool set_calibration_files(const std::string& stereo_yml, const std::string& left_mono_yml,
                               const std::string& right_mono_yml); // NOLINT(bugprone-easily-swappable-parameters)

    /**
     * @brief [en] Load the stereo pair from disk (jpg, jpeg, png or bmp)
     * @brief [zh] 从磁盘读取一对图像（jpg、jpeg、png 或 bmp）
     * @param left_path left image path / 左图路径
     * @param right_path right image path / 右图路径
     * @return true on success; a failure keeps the previously set pair
     * @since 1.0
     * @exporter
     */
    bool set_input_image(const std::string& left_path,
                         const std::string& right_path); // NOLINT(bugprone-easily-swappable-parameters)

    /**
     * @brief [en] Provide the stereo pair as pixel buffers
     * @brief [zh] 以像素缓冲区形式提供一对图像
     * @param left left image / 左图
     * @param right right image / 右图
     * @return true on success; a failure keeps the previously set pair
     * @since 1.0
     * @exporter
     */
    bool set_input_image(const Image& left, const Image& right); // NOLINT(bugprone-easily-swappable-parameters)

    /**
     * @brief [en] Rectify, match and convert to depth
     * @brief [zh] 校正、匹配并转换为深度
     * @return true on success; nothing is written to disk either way
     * @since 1.0
     * @exporter
     */
    bool run();

    /**
     * @brief [en] Disparity map of the last successful run
     * @brief [zh] 最近一次成功运行的视差图
     * @return reference to the cached map
     * @throws std::logic_error when run() has not succeeded
     * @since 1.0
     * @exporter
     */
    const DisparityImage& get_disparity_image() const;

    /**
     * @brief [en] Depth map of the last successful run
     * @brief [zh] 最近一次成功运行的深度图
     * @return reference to the cached map
     * @throws std::logic_error when run() has not succeeded
     * @since 1.0
     * @exporter
     */
    const DepthImage& get_depth_image() const;

    /**
     * @brief [en] Point cloud built from the depth map, cached after the first call
     * @brief [zh] 由深度图构建的点云，首次调用后缓存
     * @return reference to the cached cloud
     * @throws std::logic_error when run() has not succeeded
     * @since 1.0
     * @exporter
     */
    const PointCloud& get_pointcloud() const;

    /**
     * @brief [en] Write the disparity map; png is 16-bit x16 fixed point, tif is float
     * @brief [zh] 写出视差图；png 为 16 位 x16 定点，tif 为浮点
     * @param path output path / 输出路径
     * @return true on success
     * @since 1.0
     * @exporter
     */
    bool save_disparity_image(const std::string& path) const;

    /**
     * @brief [en] Write the depth map; png is 16-bit millimetres, tif is float metres
     * @brief [zh] 写出深度图；png 为 16 位毫米，tif 为浮点米
     * @param path output path / 输出路径
     * @return true on success
     * @since 1.0
     * @exporter
     */
    bool save_depth_image(const std::string& path) const;

    /**
     * @brief [en] Write the point cloud as PCD or PLY, chosen by the extension
     * @brief [zh] 按扩展名写出 PCD 或 PLY 点云
     * @param path output path / 输出路径
     * @return true on success
     * @since 1.0
     * @exporter
     */
    bool save_pointcloud(const std::string& path) const;

    /**
     * @brief [en] Rectified pair and validity mask of the last successful run
     * @brief [zh] 最近一次成功运行的校正图与有效性掩膜
     * @return reference to the rectified pair
     * @throws std::logic_error when run() has not succeeded
     * @since 1.0
     * @exporter
     */
    const RectifiedPair& rectified() const;

    /**
     * @brief [en] Timings and statistics of the last successful run
     * @brief [zh] 最近一次成功运行的耗时与统计
     * @return reference to the report
     * @since 1.0
     * @exporter
     */
    const RunReport& report() const;

    /**
     * @brief [en] Message of the most recent failure
     * @brief [zh] 最近一次失败的原因
     * @return empty when the last operation succeeded
     * @since 1.0
     * @exporter
     */
    const std::string& last_error() const;

    /**
     * @brief [en] The configuration this pipeline runs with
     * @brief [zh] 本流水线使用的配置
     * @return reference to the configuration
     * @since 1.0
     * @exporter
     */
    const Config& config() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sv
