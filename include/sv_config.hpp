// Conan::ImportStart
#pragma once

#include <string>
#include <vector>

#include "sv_rectify.hpp"
#include "sv_sgbm.hpp"
#include "sv_types.hpp"
// Conan::ImportEnd

namespace sv {

/// Default near working distance in metres, matching sgbm_config.json.
inline constexpr double kDefaultWorkingNear = 0.07;

/// Default far working distance in metres, matching sgbm_config.json.
inline constexpr double kDefaultWorkingFar = 0.25;

/// The disparity search window is a multiple of this, so it is rounded outwards.
inline constexpr int kDisparityStep = 16;

/**
 * @brief [en] Everything sgbm_config.json can say about a run
 * @brief [zh] sgbm_config.json 能描述的全部运行参数
 *
 * Every key is optional: an omitted key keeps the value below, so a partial file stays
 * valid. Precedence is defaults < top-level keys < profiles[profile].
 *
 * @since 1.0
 * @exporter
 */
struct Config {
    std::string profile;
    SgbmParams matcher;
    RectifyOptions rectify;

    std::string stereo_yml;
    std::string left_mono_yml;
    std::string right_mono_yml;

    double camera_fx = 0.0;
    double camera_baseline_m = 0.0;
    std::string camera_file;

    double working_near_m = kDefaultWorkingNear;
    double working_far_m = kDefaultWorkingFar;

    double depth_min_m = 0.0;
    double depth_max_m = 0.0;

    bool cloud_binary = true;
    bool cloud_keep_invalid = false;

    std::string output_directory;
    std::string output_prefix = "stereo";
    bool save_rectified = false;
    bool show_window = false;

    /// Keys that were present but not understood; the run continues without them.
    std::vector<std::string> warnings;
};

/**
 * @brief [en] A configuration holding only the built-in defaults
 * @brief [zh] 只包含内置默认值的配置
 * @return the default configuration
 * @since 1.0
 * @exporter
 */
Config config_default();

/**
 * @brief [en] Parse a JSON configuration, merging the selected profile over the top-level keys
 * @brief [zh] 解析 JSON 配置，并把选中的 profile 合并到顶层键之上
 * @param path JSON path / JSON 路径
 * @param config receives the parsed configuration / 接收解析结果
 * @param error optional message, written on failure / 失败时写入原因
 * @return true when the file parsed and validated
 * @since 1.0
 * @exporter
 */
bool config_load(const std::string& path, Config& config,
                 std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

/**
 * @brief [en] Fill in num_disparities from the working depth range when the config leaves it at 0
 * @brief [zh] 当配置里 num_disparities 为 0 时，按工作深度范围反推搜索窗口
 * @param geometry rectified geometry providing fx and the baseline / 提供 fx 与基线的校正几何
 * @param config configuration to complete / 待补全的配置
 * @param params receives the resolved matcher parameters / 接收解析后的匹配参数
 * @param error optional message, written on failure / 失败时写入原因
 * @return true when the window is usable
 * @since 1.0
 * @exporter
 */
bool config_resolve_window(const RectifyGeometry& geometry, const Config& config, SgbmParams& params,
                           std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

} // namespace sv
