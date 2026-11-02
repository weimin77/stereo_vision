// Conan::ImportStart
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "sv_types.hpp"
// Conan::ImportEnd

namespace sv {

/**
 * @brief [en] Aggregation mode of cv::StereoSGBM
 * @brief [zh] cv::StereoSGBM 的聚合模式
 * @since 1.0
 * @exporter
 */
enum class SgbmMode : std::uint8_t {
    Sgbm = 0,
    Hh = 1,
    Hh4 = 3,
    ThreeWay = 2,
};

/**
 * @brief [en] Parameters of the SGBM matcher, mirroring the names in sgbm_config.json
 * @brief [zh] SGBM 匹配参数，键名与 sgbm_config.json 一致
 *
 * P1 and P2 are derived from the block size, exactly as the reference pipeline does:
 * P1 = p1_factor * block_size^2 and P2 = p2_factor * block_size^2.
 *
 * @since 1.0
 * @exporter
 */
struct SgbmParams {
    int min_disparity = 0;
    int num_disparities = 0;
    int block_size = 5;
    int p1_factor = 8;
    int p2_factor = 256;
    int pre_filter_cap = 31;
    int uniqueness_ratio = 5;
    /// Handed to cv::StereoSGBM, where only MODE_HH and MODE_HH4 use it for their internal check.
    int disp12_max_diff = 1;
    int speckle_window_size = 40;
    int speckle_range = 1;
    SgbmMode mode = SgbmMode::Sgbm;
    /// Pad the images so the search window never runs off the left edge.
    bool padding = true;
    bool clahe = false;
    double clahe_clip_limit = 2.0;
    int clahe_tile_grid = 8;
};

/**
 * @brief [en] Validate the matcher parameters and resolve the search window
 * @brief [zh] 校验匹配参数并确定搜索窗口
 * @param params user parameters / 用户参数
 * @param error optional message, written on failure / 失败时写入原因
 * @return true when the parameters describe a legal run
 * @since 1.0
 * @exporter
 */
bool sgbm_validate(const SgbmParams& params, std::string* error);

/**
 * @brief [en] Compute a disparity map with cv::StereoSGBM in a single left-to-right pass
 * @brief [zh] 用 cv::StereoSGBM 单次左到右计算视差图
 *
 * The result is in pixels, at sub-pixel precision, with NaN marking pixels that fell outside
 * the search window, landed outside the right image, were masked out, or carry the matcher's
 * invalid sentinel.
 *
 * The matcher runs once. A right-to-left pass would allow a left-right consistency check, but
 * it doubles the cost of this stage, so the caller gets the raw left estimate instead.
 *
 * @param left left image, 1 or 3 channels / 左图，1 或 3 通道
 * @param right right image of the same size / 同尺寸右图
 * @param valid_mask optional mask, 255 for usable pixels; empty means "all usable"
 * @param params matcher parameters / 匹配参数
 * @param disparity receives the map / 接收视差图
 * @param error optional message, written on failure / 失败时写入原因
 * @return true on success
 * @since 1.0
 * @exporter
 */
bool sgbm_compute(const Image& left, const Image& right, const std::vector<std::uint8_t>& valid_mask,
                  const SgbmParams& params, DisparityImage& disparity, std::string* error);

} // namespace sv
