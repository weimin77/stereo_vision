// Conan::ImportStart
#include "sv_pipeline.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>
// Conan::ImportEnd

namespace sv {

namespace {

namespace fs = std::filesystem;

constexpr double kZero = 0.0;

std::string directory_of(const std::string& path) {
    const fs::path parent = fs::path(path).parent_path();
    return parent.empty() ? fs::current_path().string() : parent.string();
}

} // namespace


struct Pipeline::Impl {
    Config config;
    Calibration calibration;
    bool has_calibration = false;
    Image left;
    Image right;
    std::string left_path;
    bool has_input = false;

    Rectifier rectifier;
    RectifiedPair rectified;
    DisparityImage disparity;
    DepthImage depth;
    mutable PointCloud cloud;
    mutable bool cloud_dirty = true;
    RunReport report;
    bool has_result = false;
    std::string error;

    const Image& require_result() const {
        if (!has_result) {
            throw std::logic_error(error.empty() ? "run() has not succeeded yet" : error);
        }
        return left;
    }
};


Pipeline::Pipeline(const Config& config) : impl_(std::make_unique<Impl>()) { impl_->config = config; }

Pipeline::~Pipeline() = default;

Pipeline::Pipeline(Pipeline&& other) noexcept = default;

Pipeline& Pipeline::operator=(Pipeline&& other) noexcept = default;

const Config& Pipeline::config() const { return impl_->config; }

const std::string& Pipeline::last_error() const { return impl_->error; }

const RunReport& Pipeline::report() const { return impl_->report; }

bool Pipeline::set_calibration(const Calibration& calibration) {
    Impl& impl = *impl_;
    std::string error;
    if (!calibration_valid(calibration, &error)) {
        impl.error = "calibration rejected: " + error;
        return false;
    }
    impl.calibration = calibration;
    impl.has_calibration = true;
    impl.has_result = false;
    impl.cloud_dirty = true;
    impl.error.clear();
    return true;
}

bool Pipeline::set_calibration_files(const std::string& stereo_yml, const std::string& left_mono_yml,
                                     const std::string& right_mono_yml) { // NOLINT
    Impl& impl = *impl_;
    Calibration calibration;
    std::string error;
    if (!calibration_load_stereo(stereo_yml, calibration, &error)) {
        impl.error = error;
        return false;
    }
    if (!left_mono_yml.empty() && !calibration_override_left(left_mono_yml, calibration, &error)) {
        impl.error = error;
        return false;
    }
    if (!right_mono_yml.empty() && !calibration_override_right(right_mono_yml, calibration, &error)) {
        impl.error = error;
        return false;
    }
    return set_calibration(calibration);
}

bool Pipeline::set_input_image(const std::string& left_path, const std::string& right_path) { // NOLINT
    Impl& impl = *impl_;
    Image left;
    Image right;
    std::string error;
    if (!image_load(left_path, left, &error)) {
        impl.error = error;
        return false;
    }
    if (!image_load(right_path, right, &error)) {
        impl.error = error;
        return false;
    }
    if (!set_input_image(left, right)) {
        return false;
    }
    impl.left_path = left_path;
    return true;
}

bool Pipeline::set_input_image(const Image& left, const Image& right) { // NOLINT
    Impl& impl = *impl_;
    if (!image_valid(left) || !image_valid(right)) {
        impl.error = "input images are empty or inconsistent";
        return false;
    }
    if (left.width != right.width || left.height != right.height) {
        impl.error = "left and right images must have the same size";
        return false;
    }
    if (left.channels != right.channels) {
        impl.error = "left and right images must share one channel count";
        return false;
    }
    impl.left = left;
    impl.right = right;
    impl.left_path.clear();
    impl.has_input = true;
    impl.has_result = false;
    impl.cloud_dirty = true;
    impl.error.clear();
    return true;
}

bool Pipeline::run() {
    using clock = std::chrono::steady_clock;
    Impl& impl = *impl_;
    impl.has_result = false;
    impl.cloud_dirty = true;

    if (!impl.has_input) {
        impl.error = "no input images: call set_input_image() first";
        return false;
    }
    if (!impl.has_calibration) {
        impl.error = "no calibration: call set_calibration() or set_calibration_files() first";
        return false;
    }
    if (!image_valid(impl.left)) {
        impl.error = "left image is empty or inconsistent";
        return false;
    }

    const auto rectify_start = clock::now();
    std::string error;
    if (!impl.rectifier.prepare(impl.calibration, impl.left.width, impl.left.height, impl.config.rectify, &error)) {
        impl.error = "rectification failed: " + error;
        return false;
    }
    if (!impl.rectifier.apply(impl.left, impl.right, impl.rectified, &error)) {
        impl.error = "rectification failed: " + error;
        return false;
    }
    const double rectify_ms = std::chrono::duration<double, std::milli>(clock::now() - rectify_start).count();

    const auto match_start = clock::now();
    SgbmParams params;
    if (!config_resolve_window(impl.rectifier.geometry(), impl.config, params, &error)) {
        impl.error = "matcher parameters rejected: " + error;
        return false;
    }
    if (!sgbm_compute(impl.rectified.left, impl.rectified.right, impl.rectified.valid_mask, params, impl.disparity,
                      &error)) {
        impl.error = "matching failed: " + error;
        return false;
    }
    const double match_ms = std::chrono::duration<double, std::milli>(clock::now() - match_start).count();

    const auto depth_start = clock::now();
    if (!depth_from_disparity(impl.disparity, impl.rectifier.geometry(), impl.config.depth_min_m,
                              impl.config.depth_max_m, impl.depth, &error)) {
        impl.error = "depth conversion failed: " + error;
        return false;
    }
    const double depth_ms = std::chrono::duration<double, std::milli>(clock::now() - depth_start).count();

    const std::size_t total = impl.disparity.data.size();
    std::size_t valid = 0;
    for (const float value : impl.disparity.data) {
        if (std::isfinite(value)) {
            ++valid;
        }
    }

    RunReport& report = impl.report;
    report = RunReport();
    report.rectify_ms = rectify_ms;
    report.match_ms = match_ms;
    report.depth_ms = depth_ms;
    report.rectified_width = impl.rectifier.geometry().width;
    report.rectified_height = impl.rectifier.geometry().height;
    report.min_disparity = params.min_disparity;
    report.num_disparities = params.num_disparities;
    report.valid_ratio = total == 0 ? kZero : static_cast<double>(valid) / static_cast<double>(total);
    double disparity_percentiles[3] = {0.0, 0.0, 0.0};
    percentiles(impl.disparity.data, disparity_percentiles);
    report.disparity_p05 = disparity_percentiles[0];
    report.disparity_p50 = disparity_percentiles[1];
    report.disparity_p95 = disparity_percentiles[2];
    double depth_percentiles[3] = {0.0, 0.0, 0.0};
    percentiles(impl.depth.data, depth_percentiles);
    report.depth_p05 = depth_percentiles[0];
    report.depth_p50 = depth_percentiles[1];
    report.depth_p95 = depth_percentiles[2];
    report.output_directory_resolved =
        impl.config.output_directory.empty() ? directory_of(impl.left_path) : impl.config.output_directory;

    impl.has_result = true;
    impl.error.clear();
    return true;
}

const DisparityImage& Pipeline::get_disparity_image() const {
    impl_->require_result();
    return impl_->disparity;
}

const DepthImage& Pipeline::get_depth_image() const {
    impl_->require_result();
    return impl_->depth;
}

const PointCloud& Pipeline::get_pointcloud() const {
    Impl& impl = *impl_;
    impl.require_result();
    if (impl.cloud_dirty) {
        std::string error;
        PointCloud cloud;
        if (!pointcloud_from_depth(impl.depth, impl.rectifier.geometry(), false, cloud, &error)) {
            throw std::logic_error("point cloud build failed: " + error);
        }
        impl.cloud = std::move(cloud);
        impl.cloud_dirty = false;
    }
    return impl.cloud;
}

bool Pipeline::save_disparity_image(const std::string& path, bool colorize) const {
    Impl& impl = *impl_;
    impl.require_result();
    std::string error;
    if (!disparity_save(path, impl.disparity, &error, colorize)) {
        impl.error = error;
        return false;
    }
    impl.error.clear();
    return true;
}

bool Pipeline::save_depth_image(const std::string& path, bool colorize) const {
    Impl& impl = *impl_;
    impl.require_result();
    std::string error;
    if (!depth_save(path, impl.depth, &error, colorize)) {
        impl.error = error;
        return false;
    }
    impl.error.clear();
    return true;
}

bool Pipeline::save_pointcloud(const std::string& path) const {
    Impl& impl = *impl_;
    impl.require_result();
    std::string error;
    PointCloud cloud;
    if (!pointcloud_from_depth(impl.depth, impl.rectifier.geometry(), impl.config.cloud_keep_invalid, cloud, &error)) {
        impl.error = error;
        return false;
    }
    if (!pointcloud_save(path, cloud, impl.config.cloud_binary, &error)) {
        impl.error = error;
        return false;
    }
    impl.error.clear();
    return true;
}

const RectifiedPair& Pipeline::rectified() const {
    impl_->require_result();
    return impl_->rectified;
}

} // namespace sv
