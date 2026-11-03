// Conan::ImportStart
#include "sv_sgbm.hpp"

#include <limits>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
// Conan::ImportEnd

namespace sv {

namespace {

constexpr int kDisparityBlockMultiple = 16;
constexpr int kBlockMin = 3;
constexpr int kFixedPointScale = 16;
constexpr int kChannelsGray = 1;
constexpr int kChannelsColor = 3;
constexpr double kInvalid = std::numeric_limits<double>::quiet_NaN();
/// Stands in for "no right match here"; also the value remap uses outside the right map.
constexpr float kRightMissing = -1.0e9F;

void set_error(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

int to_cv_mode(SgbmMode mode) {
    switch (mode) {
    case SgbmMode::Hh:
        return cv::StereoSGBM::MODE_HH;
    case SgbmMode::Hh4:
        return cv::StereoSGBM::MODE_HH4;
    case SgbmMode::ThreeWay:
        return cv::StereoSGBM::MODE_SGBM_3WAY;
    case SgbmMode::Sgbm:
    default:
        return cv::StereoSGBM::MODE_SGBM;
    }
}

cv::Ptr<cv::StereoSGBM> make_matcher(const SgbmParams& params, int min_disparity) {
    const int block = params.block_size;
    const int p1 = params.p1_factor * block * block;
    const int p2 = params.p2_factor * block * block;
    return cv::StereoSGBM::create(min_disparity, params.num_disparities, block, p1, p2, params.disp12_max_diff,
                                  params.pre_filter_cap, params.uniqueness_ratio, params.speckle_window_size,
                                  params.speckle_range, to_cv_mode(params.mode));
}

cv::Mat to_gray(const Image& image) {
    const int type = image.channels == kChannelsGray ? CV_8UC1 : CV_8UC3;
    cv::Mat mat(image.height, image.width, type);
    std::copy(image.data.begin(), image.data.end(), mat.ptr<std::uint8_t>());
    if (image.channels == kChannelsGray) {
        return mat;
    }
    cv::Mat gray;
    cv::cvtColor(mat, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

} // namespace


bool sgbm_validate(const SgbmParams& params, std::string* error) {
    if (params.num_disparities <= 0 || params.num_disparities % kDisparityBlockMultiple != 0) {
        set_error(error,
                  "num_disparities must be a positive multiple of 16, got " + std::to_string(params.num_disparities));
        return false;
    }
    if (params.min_disparity < 0) {
        set_error(error, "min_disparity must be >= 0, got " + std::to_string(params.min_disparity));
        return false;
    }
    if (params.block_size < kBlockMin || params.block_size % 2 == 0) {
        set_error(error, "block_size must be an odd number >= 3, got " + std::to_string(params.block_size));
        return false;
    }
    if (params.p1_factor <= 0 || params.p2_factor <= params.p1_factor) {
        set_error(error, "p2_factor must be greater than p1_factor and both positive");
        return false;
    }
    if (params.clahe && params.clahe_tile_grid <= 0) {
        set_error(error, "clahe_tile_grid must be positive");
        return false;
    }
    if (!(params.lr_max_diff_px >= 0.0)) {
        set_error(error, "lr_max_diff_px must be >= 0");
        return false;
    }
    return true;
}

bool sgbm_compute(const Image& left, const Image& right, const std::vector<std::uint8_t>& valid_mask,
                  const SgbmParams& params, DisparityImage& disparity, std::string* error) {
    if (!sgbm_validate(params, error)) {
        return false;
    }
    if (!image_valid(left) || !image_valid(right)) {
        set_error(error, "both images must be non-empty");
        return false;
    }
    if (left.width != right.width || left.height != right.height) {
        set_error(error, "left and right images must share one size");
        return false;
    }
    const int width = left.width;
    const int height = left.height;
    if (params.num_disparities >= width) {
        set_error(error, "num_disparities must be smaller than the image width");
        return false;
    }
    if (!valid_mask.empty() && valid_mask.size() != static_cast<std::size_t>(width) * height) {
        set_error(error, "valid_mask must match the image size");
        return false;
    }

    const int padding = params.padding ? (params.min_disparity + params.num_disparities + params.block_size) : 0;
    if (padding >= width || padding >= height) {
        set_error(error, "the disparity search window does not fit the image at this padding");
        return false;
    }

    cv::Mat left_gray = to_gray(left);
    cv::Mat right_gray = to_gray(right);
    if (params.clahe) {
        const cv::Size tile(params.clahe_tile_grid, params.clahe_tile_grid);
        const auto clahe_left = cv::createCLAHE(params.clahe_clip_limit, tile);
        const auto clahe_right = cv::createCLAHE(params.clahe_clip_limit, tile);
        clahe_left->apply(left_gray, left_gray);
        clahe_right->apply(right_gray, right_gray);
    }
    cv::Mat left_padded;
    cv::Mat right_padded;
    cv::copyMakeBorder(left_gray, left_padded, 0, 0, padding, padding, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::copyMakeBorder(right_gray, right_padded, 0, 0, padding, padding, cv::BORDER_CONSTANT, cv::Scalar(0));

    // The right view searches the mirrored window, so its disparities come out negative.
    const int right_min_disparity = -params.min_disparity - params.num_disparities + 1;
    const int left_sentinel = (params.min_disparity - 1) * kFixedPointScale;
    const int right_sentinel = (right_min_disparity - 1) * kFixedPointScale;

    cv::Mat left_fixed;
    cv::Mat right_fixed;
    make_matcher(params, params.min_disparity)->compute(left_padded, right_padded, left_fixed);
    make_matcher(params, right_min_disparity)->compute(right_padded, left_padded, right_fixed);

    const cv::Rect crop(padding, 0, width, height);
    cv::Mat left_cropped = left_fixed(crop);
    cv::Mat right_cropped = right_fixed(crop);

    // 1/16 px fixed point to pixels. 1/16 is a power of two, so the scale introduces no rounding.
    cv::Mat left_values;
    cv::Mat right_values;
    left_cropped.convertTo(left_values, CV_32F, 1.0 / kFixedPointScale);
    right_cropped.convertTo(right_values, CV_32F, 1.0 / kFixedPointScale);

    // Values outside the window the caller asked for are search artefacts, not measurements, and
    // so is anything under the caller's mask. The right map only keeps the sentinel test: its
    // values are read at the left pixel's own match, not at the pixel of the same index.
    const float limit_low = static_cast<float>(params.min_disparity);
    const float limit_high =
        static_cast<float>(params.min_disparity + params.num_disparities) - (1.0F / kFixedPointScale);
    cv::Mat usable;
    cv::bitwise_and(left_cropped > left_sentinel, left_values >= limit_low, usable);
    cv::bitwise_and(usable, left_values <= limit_high, usable);
    if (!valid_mask.empty()) {
        const cv::Mat mask(height, width, CV_8UC1, const_cast<std::uint8_t*>(valid_mask.data()));
        cv::bitwise_and(usable, mask != 0, usable);
    }
    right_values.setTo(kRightMissing, right_cropped <= right_sentinel);

    // Left-right consistency as one bilinear gather instead of a per-pixel loop: the left estimate
    // says where its partner is, so the right map is sampled there and the two are compared in
    // disparity space. Sampling off the right map yields kRightMissing, which fails both tests.
    cv::Mat columns(1, width, CV_32F);
    cv::Mat rows(height, 1, CV_32F);
    for (int x = 0; x < width; ++x) {
        columns.at<float>(0, x) = static_cast<float>(x);
    }
    for (int y = 0; y < height; ++y) {
        rows.at<float>(y, 0) = static_cast<float>(y);
    }
    cv::Mat map_x;
    cv::repeat(columns, height, 1, map_x);
    cv::subtract(map_x, left_values, map_x);
    cv::patchNaNs(map_x, kRightMissing);
    cv::Mat map_y;
    cv::repeat(rows, 1, width, map_y);
    cv::Mat sampled;
    cv::remap(right_values, sampled, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(kRightMissing));
    map_x.release();
    map_y.release();
    right_values.release();

    // A consistent pair sums to zero: the left estimate plus the right one it points at.
    cv::Mat paired;
    cv::add(left_values, sampled, paired);
    cv::Mat magnitude;
    cv::absdiff(paired, cv::Scalar(0), magnitude);
    cv::Mat accepted;
    cv::compare(magnitude, params.lr_max_diff_px, accepted, cv::CMP_LE);
    cv::compare(sampled, 0.0, magnitude, cv::CMP_LT);
    cv::bitwise_and(accepted, magnitude, accepted);
    cv::bitwise_and(accepted, usable, accepted);

    sampled.setTo(kInvalid);
    left_values.copyTo(sampled, accepted);

    disparity.width = width;
    disparity.height = height;
    disparity.data.assign(sampled.ptr<float>(), sampled.ptr<float>() + (static_cast<std::size_t>(width) * height));
    return true;
}

} // namespace sv
