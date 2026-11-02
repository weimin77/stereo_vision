// Conan::ImportStart
#include "sv_config.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
// Conan::ImportEnd

namespace sv {

namespace {

constexpr double kZero = 0.0;
constexpr int kSequenceSize = 2;
constexpr double kWindowSlack = 1.0;

void set_error(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

std::string lower(const std::string& text) {
    std::string out = text;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

/// Keys this schema understands; anything else is reported and ignored, never fatal.
const std::set<std::string>& known_keys() {
    static const std::set<std::string> keys = {
        "profile",          "profiles",       "calibration",      "rectify",
        "working_depth_m",  "min_disparity",  "num_disparities",  "padding",
        "block_size",       "p1_factor",      "p2_factor",        "mode",
        "uniqueness_ratio", "pre_filter_cap", "disp12_max_diff",  "speckle_window_size",
        "speckle_range",    "preprocess",     "clahe_clip_limit", "clahe_tile_grid",
        "median_size",      "camera",         "camera_file",      "depth",
        "pointcloud",       "output"};
    return keys;
}

bool read_int(const cv::FileNode& parent, const char* key, int& out) {
    const cv::FileNode node = parent[key];
    if (node.empty() || (!node.isInt() && !node.isReal())) {
        return false;
    }
    out = static_cast<int>(static_cast<double>(node.real()));
    return true;
}

bool read_double(const cv::FileNode& parent, const char* key, double& out) {
    const cv::FileNode node = parent[key];
    if (node.empty() || (!node.isReal() && !node.isInt())) {
        return false;
    }
    out = static_cast<double>(node.real());
    return true;
}

bool read_bool(const cv::FileNode& parent, const char* key, bool& out) {
    const cv::FileNode node = parent[key];
    if (node.empty()) {
        return false;
    }
    if (node.isInt() || node.isReal()) {
        out = static_cast<int>(static_cast<double>(node.real())) != 0;
        return true;
    }
    if (node.isString()) {
        const std::string text = lower(static_cast<std::string>(node.string()));
        if (text == "true" || text == "yes" || text == "1") {
            out = true;
            return true;
        }
        if (text == "false" || text == "no" || text == "0") {
            out = false;
            return true;
        }
    }
    return false;
}

bool read_string(const cv::FileNode& parent, const char* key, std::string& out) {
    const cv::FileNode node = parent[key];
    if (node.empty() || !node.isString()) {
        return false;
    }
    out = static_cast<std::string>(node.string());
    return true;
}

bool read_sequence(const cv::FileNode& parent, const char* key, double (&out)[kSequenceSize]) { // NOLINT
    const cv::FileNode node = parent[key];
    if (node.empty() || !node.isSeq() || node.size() != kSequenceSize) {
        return false;
    }
    for (int i = 0; i < kSequenceSize; ++i) {
        const cv::FileNode item = node[i];
        if (!item.isReal() && !item.isInt()) {
            return false;
        }
        out[i] = static_cast<double>(item.real());
    }
    return true;
}

bool parse_mode(const std::string& text, SgbmMode& mode) {
    const std::string value = lower(text);
    if (value == "sgbm") {
        mode = SgbmMode::Sgbm;
    } else if (value == "hh") {
        mode = SgbmMode::Hh;
    } else if (value == "hh4") {
        mode = SgbmMode::Hh4;
    } else if (value == "3way") {
        mode = SgbmMode::ThreeWay;
    } else {
        return false;
    }
    return true;
}

/// Keys a profile may override: only the matcher block, as in the reference pipeline.
void apply_matcher(const cv::FileNode& node, Config& config) {
    read_int(node, "min_disparity", config.matcher.min_disparity);
    read_int(node, "num_disparities", config.matcher.num_disparities);
    read_int(node, "block_size", config.matcher.block_size);
    read_int(node, "p1_factor", config.matcher.p1_factor);
    read_int(node, "p2_factor", config.matcher.p2_factor);
    read_int(node, "uniqueness_ratio", config.matcher.uniqueness_ratio);
    read_int(node, "pre_filter_cap", config.matcher.pre_filter_cap);
    read_int(node, "disp12_max_diff", config.matcher.disp12_max_diff);
    read_int(node, "speckle_window_size", config.matcher.speckle_window_size);
    read_int(node, "speckle_range", config.matcher.speckle_range);
    read_bool(node, "padding", config.matcher.padding);

    std::string mode;
    if (read_string(node, "mode", mode) && !parse_mode(mode, config.matcher.mode)) {
        config.warnings.emplace_back("unknown mode '" + mode + "', keeping the current one");
    }
    std::string preprocess;
    if (read_string(node, "preprocess", preprocess)) {
        const std::string value = lower(preprocess);
        if (value == "clahe") {
            config.matcher.clahe = true;
        } else if (value == "none") {
            config.matcher.clahe = false;
        } else {
            config.warnings.emplace_back("unknown preprocess '" + preprocess + "', keeping the current one");
        }
    }
}

void apply_rectify(const cv::FileNode& node, Config& config) {
    read_bool(node, "enabled", config.rectify.enabled);
    read_double(node, "alpha", config.rectify.alpha);
    read_bool(node, "crop_to_common_roi", config.rectify.crop_to_common_roi);
    read_int(node, "new_width", config.rectify.new_width);
    read_int(node, "new_height", config.rectify.new_height);
    double size[kSequenceSize] = {0.0, 0.0};
    if (read_sequence(node, "new_size", size)) {
        config.rectify.new_width = static_cast<int>(size[0]);
        config.rectify.new_height = static_cast<int>(size[1]);
    }
    std::string map_type;
    if (read_string(node, "map_type", map_type)) {
        const std::string value = lower(map_type);
        if (value == "float") {
            config.rectify.fixed_point_maps = false;
        } else if (value == "fixed") {
            config.rectify.fixed_point_maps = true;
        } else {
            config.warnings.emplace_back("unknown rectify.map_type '" + map_type + "', keeping fixed");
        }
    }
}

void apply_calibration_paths(const cv::FileNode& node, Config& config) {
    read_string(node, "stereo_yml", config.stereo_yml);
    read_string(node, "left_mono_yml", config.left_mono_yml);
    read_string(node, "right_mono_yml", config.right_mono_yml);
}

void apply_camera(const cv::FileNode& node, Config& config) {
    read_double(node, "fx", config.camera_fx);
    read_double(node, "baseline_m", config.camera_baseline_m);
}

void apply_depth(const cv::FileNode& node, Config& config) {
    read_double(node, "min_m", config.depth_min_m);
    read_double(node, "max_m", config.depth_max_m);
}

void apply_pointcloud(const cv::FileNode& node, Config& config) {
    std::string encoding;
    if (read_string(node, "encoding", encoding)) {
        const std::string value = lower(encoding);
        if (value == "binary") {
            config.cloud_binary = true;
        } else if (value == "ascii") {
            config.cloud_binary = false;
        } else {
            config.warnings.emplace_back("unknown pointcloud.encoding '" + encoding + "', keeping binary");
        }
    }
    read_bool(node, "keep_invalid", config.cloud_keep_invalid);
}

void apply_output(const cv::FileNode& node, Config& config) {
    read_string(node, "directory", config.output_directory);
    read_string(node, "prefix", config.output_prefix);
    read_bool(node, "save_rectified", config.save_rectified);
    read_bool(node, "show_window", config.show_window);
}

void apply_root(const cv::FileNode& root, Config& config) {
    double depth_range[kSequenceSize] = {config.working_near_m, config.working_far_m};
    if (read_sequence(root, "working_depth_m", depth_range)) {
        config.working_near_m = depth_range[0];
        config.working_far_m = depth_range[1];
    }
    apply_matcher(root, config);

    std::string preprocess;
    if (read_string(root, "preprocess", preprocess)) {
        const std::string value = lower(preprocess);
        if (value == "clahe") {
            config.matcher.clahe = true;
        } else if (value == "none") {
            config.matcher.clahe = false;
        } else {
            config.warnings.emplace_back("unknown preprocess '" + preprocess + "', keeping the current one");
        }
    }
    read_double(root, "clahe_clip_limit", config.matcher.clahe_clip_limit);
    read_int(root, "clahe_tile_grid", config.matcher.clahe_tile_grid);

    apply_calibration_paths(root["calibration"], config);
    apply_rectify(root["rectify"], config);
    apply_camera(root["camera"], config);
    read_string(root, "camera_file", config.camera_file);
    apply_depth(root["depth"], config);
    apply_pointcloud(root["pointcloud"], config);
    apply_output(root["output"], config);
}

} // namespace


Config config_default() { return Config(); }

bool config_load(const std::string& path, Config& config, std::string* error) { // NOLINT
    config = Config();

    cv::FileStorage storage;
    storage.open(path, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        set_error(error, "cannot open configuration: " + path);
        return false;
    }
    const cv::FileNode root = storage.root();
    if (root.empty()) {
        set_error(error, "configuration is empty: " + path);
        return false;
    }
    if (!root.isMap()) {
        set_error(error, "configuration must be a JSON object: " + path);
        return false;
    }

    for (auto it = root.begin(); it != root.end(); ++it) {
        const std::string key = (*it).name();
        // Keys beginning with '_' are documentation for the reader, not parameters.
        if (key.empty() || key[0] == '_' || known_keys().count(key) != 0) {
            continue;
        }
        config.warnings.emplace_back("ignoring unknown key '" + key + "'");
    }

    read_string(root, "profile", config.profile);
    apply_root(root, config);

    if (!config.profile.empty()) {
        const cv::FileNode profile = root["profiles"][config.profile];
        if (profile.empty()) {
            config.warnings.emplace_back("profile '" + config.profile + "' is not defined, using the top-level values");
        } else {
            apply_matcher(profile, config);
        }
    }

    if (config.matcher.num_disparities > 0 && config.matcher.num_disparities % kDisparityStep != 0) {
        set_error(error,
                  "num_disparities must be a multiple of 16, got " + std::to_string(config.matcher.num_disparities));
        return false;
    }
    // num_disparities == 0 is legal here: config_resolve_window() derives it from
    // working_depth_m once the rectified focal baseline is known.
    SgbmParams probe = config.matcher;
    if (probe.num_disparities == 0) {
        probe.num_disparities = kDisparityStep;
    }
    return sgbm_validate(probe, error);
}

bool config_resolve_window(const RectifyGeometry& geometry, const Config& config, SgbmParams& params,
                           std::string* error) { // NOLINT
    params = config.matcher;
    if (params.num_disparities > 0) {
        if (params.min_disparity < 0) {
            set_error(error, "min_disparity must be >= 0 when num_disparities is given");
            return false;
        }
        return true;
    }

    if (!(geometry.focal_baseline > kZero)) {
        set_error(error, "a working-depth search window needs a positive focal baseline");
        return false;
    }
    if (!(config.working_near_m > kZero) || !(config.working_far_m > config.working_near_m)) {
        set_error(error, "working_depth_m must satisfy 0 < near < far");
        return false;
    }

    const double low = geometry.focal_baseline / config.working_far_m;
    const double high = geometry.focal_baseline / config.working_near_m;
    const double step = static_cast<double>(kDisparityStep);
    params.min_disparity = static_cast<int>(std::floor(low / step)) * kDisparityStep;
    const double span = std::ceil((high - static_cast<double>(params.min_disparity) + kWindowSlack) / step) * step;
    if (span < step || span > static_cast<double>(std::numeric_limits<int>::max())) {
        set_error(error, "working_depth_m yields an empty or oversized disparity range");
        return false;
    }
    params.num_disparities = static_cast<int>(span);
    return sgbm_validate(params, error);
}

} // namespace sv
