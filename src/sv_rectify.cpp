// Conan::ImportStart
#include "sv_rectify.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
// Conan::ImportEnd

namespace sv {

namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kAlphaMin = 0.0;
constexpr double kAlphaMax = 1.0;
constexpr int kMatrixRows = 3;
constexpr int kMatrixCols = 3;
constexpr int kProjectionRows = 3;
constexpr int kProjectionCols = 4;
constexpr int kHomogeneousRows = 4;
constexpr int kHomogeneousCols = 4;
constexpr int kMaskErodeIterations = 1;
constexpr int kMaskErodeKernel = 3;
constexpr int kValidMaskValue = 255;
constexpr int kDisparityOffsetIndex = 3;

void set_error(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

cv::Mat to_mat(const double* values, int rows, int cols) {
    cv::Mat matrix(rows, cols, CV_64F);
    std::copy(values, values + (rows * cols), matrix.ptr<double>());
    return matrix;
}

cv::Mat to_row(const std::vector<double>& values) {
    cv::Mat matrix(1, static_cast<int>(values.size()), CV_64F);
    std::copy(values.begin(), values.end(), matrix.ptr<double>());
    return matrix;
}

cv::Mat to_cv(const Image& image) {
    const int type = image.channels == 1 ? CV_8UC1 : CV_8UC3;
    cv::Mat mat(image.height, image.width, type);
    std::copy(image.data.begin(), image.data.end(), mat.ptr<std::uint8_t>());
    return mat;
}

void from_cv(const cv::Mat& mat, Image& image) {
    image.width = mat.cols;
    image.height = mat.rows;
    image.channels = mat.channels();
    cv::Mat continuous = mat.isContinuous() ? mat : mat.clone();
    image.data.assign(continuous.data,
                      continuous.data + (continuous.total() * static_cast<std::size_t>(continuous.channels())));
}

cv::Rect intersect_rect(const cv::Rect& a, const cv::Rect& b) { return a & b; }

/// Rebuild the standard Q from the rectified geometry when the file has none.
void rebuild_q(RectifyGeometry& geometry, double translation_x) {
    geometry.q.fill(0.0);
    geometry.q[0] = 1.0;
    geometry.q[5] = 1.0;
    geometry.q[2] = -geometry.cx;
    geometry.q[6] = -geometry.cy;
    geometry.q[10] = geometry.fx;
    if (std::abs(translation_x) > kEpsilon) {
        geometry.q[11] = -1.0 / translation_x;
        geometry.q[15] = geometry.doffs / translation_x;
    }
}

} // namespace


struct Rectifier::Impl {
    RectifyGeometry geometry;
    bool ready = false;
    bool enabled = true;
    bool identity = false;
    int input_width = 0;
    int input_height = 0;
    cv::Mat map1x;
    cv::Mat map1y;
    cv::Mat map2x;
    cv::Mat map2y;
    cv::Mat mask;
};


Rectifier::Rectifier() : impl_(std::make_unique<Impl>()) {}

Rectifier::~Rectifier() = default;

Rectifier::Rectifier(Rectifier&& other) noexcept = default;

Rectifier& Rectifier::operator=(Rectifier&& other) noexcept = default;

const RectifyGeometry& Rectifier::geometry() const { return impl_->geometry; }

bool Rectifier::prepared() const { return impl_->ready; }

void Rectifier::reset() { impl_ = std::make_unique<Impl>(); }

bool Rectifier::prepare(const Calibration& calibration, int width, int height, const RectifyOptions& options,
                        std::string* error) { // NOLINT
    impl_ = std::make_unique<Impl>();
    if (width <= 0 || height <= 0) {
        set_error(error, "rectification needs a positive image size");
        return false;
    }
    if (options.alpha < kAlphaMin || options.alpha > kAlphaMax) {
        set_error(error, "rectify.alpha must be within [0, 1]");
        return false;
    }

    Impl& impl = *impl_;
    impl.enabled = options.enabled;
    impl.input_width = width;
    impl.input_height = height;

    const cv::Mat camera1 = to_mat(calibration.m1.data(), kMatrixRows, kMatrixCols);
    const cv::Mat camera2 = to_mat(calibration.m2.data(), kMatrixRows, kMatrixCols);
    const cv::Mat dist1 = to_row(calibration.d1);
    const cv::Mat dist2 = to_row(calibration.d2);

    if (!impl.enabled) {
        // Already rectified input: the intrinsics come from the left camera, no maps are needed.
        RectifyGeometry& geometry = impl.geometry;
        geometry.width = width;
        geometry.height = height;
        geometry.fx = calibration.m1[0];
        geometry.fy = calibration.m1[4];
        geometry.cx = calibration.m1[2];
        geometry.cy = calibration.m1[5];
        geometry.doffs = 0.0;
        if (calibration.has_extrinsics) {
            const double tx = calibration.t[0];
            const double ty = calibration.t[1];
            const double tz = calibration.t[2];
            geometry.baseline = std::sqrt((tx * tx) + (ty * ty) + (tz * tz));
        } else if (calibration.has_rectification) {
            geometry.baseline = std::abs(calibration.p2[kDisparityOffsetIndex]) / geometry.fx;
        }
        if (!(geometry.baseline > 0.0)) {
            set_error(error, "rectification is disabled but no baseline is available in the calibration");
            return false;
        }
        geometry.focal_baseline = geometry.fx * geometry.baseline;
        rebuild_q(geometry, -geometry.baseline);

        impl.mask = cv::Mat(height, width, CV_8UC1, cv::Scalar(kValidMaskValue));
        impl.identity = true;
        impl.ready = true;
        return true;
    }

    cv::Mat rot1;
    cv::Mat rot2;
    cv::Mat proj1;
    cv::Mat proj2;
    cv::Mat q;
    cv::Rect roi1;
    cv::Rect roi2;
    const cv::Size input_size(width, height);
    const cv::Size full_size = (options.new_width > 0 && options.new_height > 0)
                                   ? cv::Size(options.new_width, options.new_height)
                                   : input_size;

    if (calibration.has_extrinsics) {
        cv::stereoRectify(camera1, dist1, camera2, dist2, input_size,
                          to_mat(calibration.r.data(), kMatrixRows, kMatrixCols),
                          to_mat(calibration.t.data(), static_cast<int>(kVector3Size), 1), rot1, rot2, proj1, proj2, q,
                          cv::CALIB_ZERO_DISPARITY, options.alpha, full_size, &roi1, &roi2);
    } else if (calibration.has_rectification) {
        rot1 = to_mat(calibration.r1.data(), kMatrixRows, kMatrixCols);
        rot2 = to_mat(calibration.r2.data(), kMatrixRows, kMatrixCols);
        proj1 = to_mat(calibration.p1.data(), kProjectionRows, kProjectionCols);
        proj2 = to_mat(calibration.p2.data(), kProjectionRows, kProjectionCols);
        if (!calibration.q.empty() &&
            std::any_of(calibration.q.begin(), calibration.q.end(), [](double v) { return v != 0.0; })) {
            q = to_mat(calibration.q.data(), kHomogeneousRows, kHomogeneousCols);
        }
        if (full_size != input_size) {
            set_error(
                error,
                "a stored rectification is tied to the calibration image size, so rectify.new_size must stay empty");
            return false;
        }
    } else {
        set_error(error, "calibration has neither R/T nor R1/R2/P1/P2, so no rectification is possible");
        return false;
    }

    cv::Rect crop(0, 0, full_size.width, full_size.height);
    if (options.crop_to_common_roi && !roi1.empty() && !roi2.empty()) {
        const cv::Rect common = intersect_rect(roi1, roi2);
        if (common.empty()) {
            set_error(error, "the two views share no valid region at alpha=" + std::to_string(options.alpha));
            return false;
        }
        crop = common;
    }

    // A crop only moves the origin: focal lengths and the baseline column stay, the principal points shift.
    proj1.at<double>(0, 2) -= static_cast<double>(crop.x);
    proj1.at<double>(1, 2) -= static_cast<double>(crop.y);
    proj2.at<double>(0, 2) -= static_cast<double>(crop.x);
    proj2.at<double>(1, 2) -= static_cast<double>(crop.y);

    RectifyGeometry& geometry = impl.geometry;
    geometry.width = crop.width;
    geometry.height = crop.height;
    geometry.fx = proj1.at<double>(0, 0);
    geometry.fy = proj1.at<double>(1, 1);
    geometry.cx = proj1.at<double>(0, 2);
    geometry.cy = proj1.at<double>(1, 2);
    geometry.doffs = proj1.at<double>(0, 2) - proj2.at<double>(0, 2);

    const double translation_x = proj2.at<double>(0, kDisparityOffsetIndex) / geometry.fx;
    if (translation_x > kEpsilon) {
        set_error(error,
                  "calibration puts the right camera on the +x side (Tx=" + std::to_string(translation_x) +
                      "), so its disparity is negative; swap the left/right inputs and their intrinsics, or pass an "
                      "already rectified calibration");
        return false;
    }
    geometry.baseline = std::abs(proj2.at<double>(0, kDisparityOffsetIndex)) / geometry.fx;
    if (!(geometry.fx > 0.0) || !(geometry.baseline > 0.0)) {
        set_error(error, "rectification produced a non-positive focal length or baseline");
        return false;
    }
    geometry.focal_baseline = geometry.fx * geometry.baseline;

    if (!q.empty() && q.rows == kHomogeneousRows && q.cols == kHomogeneousCols) {
        q.copyTo(cv::Mat(kHomogeneousRows, kHomogeneousCols, CV_64F, geometry.q.data()));
        geometry.q[3] += static_cast<double>(crop.x);
        geometry.q[7] += static_cast<double>(crop.y);
    } else {
        rebuild_q(geometry, translation_x);
    }

    const int map_type = options.fixed_point_maps ? CV_16SC2 : CV_32FC1;
    cv::Mat full1x;
    cv::Mat full1y;
    cv::Mat full2x;
    cv::Mat full2y;
    cv::initUndistortRectifyMap(camera1, dist1, rot1, proj1, full_size, map_type, full1x, full1y);
    cv::initUndistortRectifyMap(camera2, dist2, rot2, proj2, full_size, map_type, full2x, full2y);

    const cv::Rect full_rect(0, 0, full_size.width, full_size.height);

    // The same maps applied to a white image mark the pixels that really carry source data;
    // without it the matcher cannot tell a black border from a black object. Built before the
    // maps are sliced or moved, because slicing would lose the uncropped coordinates.
    const cv::Mat white(full_size, CV_8UC1, cv::Scalar(kValidMaskValue));
    cv::Mat valid1;
    cv::Mat valid2;
    cv::remap(white, valid1, full1x, full1y, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::remap(white, valid2, full2x, full2y, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
    impl.mask = (crop == full_rect) ? (valid1 & valid2) : (valid1(crop) & valid2(crop));
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kMaskErodeKernel, kMaskErodeKernel));
    cv::erode(impl.mask, impl.mask, kernel, cv::Point(-1, -1), kMaskErodeIterations);

    if (crop == full_rect) {
        impl.map1x = std::move(full1x);
        impl.map1y = std::move(full1y);
        impl.map2x = std::move(full2x);
        impl.map2y = std::move(full2y);
    } else {
        impl.map1x = full1x(crop).clone();
        impl.map1y = full1y(crop).clone();
        impl.map2x = full2x(crop).clone();
        impl.map2y = full2y(crop).clone();
    }

    impl.ready = true;
    return true;
}

bool Rectifier::apply(const Image& left, const Image& right, RectifiedPair& pair, std::string* error) const { // NOLINT
    const Impl& impl = *impl_;
    if (!impl.ready) {
        set_error(error, "the rectifier has not been prepared");
        return false;
    }
    if (!image_valid(left) || !image_valid(right)) {
        set_error(error, "both input images must be non-empty");
        return false;
    }
    if (left.width != impl.input_width || left.height != impl.input_height || right.width != left.width ||
        right.height != left.height) {
        set_error(error, "input images must have the same size the rectifier was prepared for");
        return false;
    }
    if (left.channels != right.channels) {
        set_error(error, "left and right images must have the same channel count");
        return false;
    }

    const cv::Mat source_left = to_cv(left);
    const cv::Mat source_right = to_cv(right);

    if (impl.identity) {
        pair.left = left;
        pair.right = right;
    } else {
        cv::Mat rectified_left;
        cv::Mat rectified_right;
        cv::remap(source_left, rectified_left, impl.map1x, impl.map1y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::remap(source_right, rectified_right, impl.map2x, impl.map2y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        from_cv(rectified_left, pair.left);
        from_cv(rectified_right, pair.right);
    }

    pair.valid_mask.assign(impl.mask.data, impl.mask.data + (impl.mask.total() * impl.mask.channels()));
    return true;
}

} // namespace sv
