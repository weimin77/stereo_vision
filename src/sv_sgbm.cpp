// Conan::ImportStart
#include "sv_sgbm.hpp"

#include <algorithm>
#include <cmath>
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

/// Disparity is 1/16 px fixed point; the sentinel one step below the search window is not a value.
inline bool invalid_sentinel(int value, int sentinel) { return value <= sentinel; }

inline void unpack(const cv::Mat& fixed, cv::Mat& values, double scale) {
    for (int y = 0; y < fixed.rows; ++y) {
        const auto* in = fixed.ptr<short>(y);
        auto* out = values.ptr<float>(y);
        for (int x = 0; x < fixed.cols; ++x) {
            out[x] = static_cast<float>(static_cast<double>(in[x]) / scale);
        }
    }
}

/// A disparity only means something when its right match lands inside the image.
inline bool correspondence_valid(float value, int x, int width, double low, double high) {
    if (!std::isfinite(value) || value < low || value > high) {
        return false;
    }
    const float destination = static_cast<float>(x) - value;
    return destination >= 0.0F && destination <= static_cast<float>(width - 1);
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

    const int left_sentinel = (params.min_disparity - 1) * kFixedPointScale;

    // One compute call: the left view only. A right-to-left pass would buy a left-right consistency
    // check at the price of doubling the match stage, which measured 182 ms against 94 ms on the
    // 640x480 oven pair — the check is not worth a second full search.
    cv::Mat left_fixed;
    make_matcher(params, params.min_disparity)->compute(left_padded, right_padded, left_fixed);

    const cv::Rect crop(padding, 0, width, height);
    cv::Mat left_cropped = left_fixed(crop);

    cv::Mat left_values(height, width, CV_32F);
    unpack(left_cropped, left_values, kFixedPointScale);

    // Values outside the window the caller asked for are search artefacts, not measurements.
    const double limit_low = static_cast<double>(params.min_disparity);
    const double limit_high =
        static_cast<double>(params.min_disparity + params.num_disparities) - (1.0 / kFixedPointScale);

    for (int y = 0; y < height; ++y) {
        const auto* left_raw = left_cropped.ptr<short>(y);
        const auto* mask_row = valid_mask.empty() ? nullptr : &valid_mask[static_cast<std::size_t>(y) * width];
        auto* left_row = left_values.ptr<float>(y);
        for (int x = 0; x < width; ++x) {
            const bool masked_out = mask_row != nullptr && mask_row[x] == 0;
            if (invalid_sentinel(left_raw[x], left_sentinel) || masked_out ||
                !correspondence_valid(left_row[x], x, width, limit_low, limit_high)) {
                left_row[x] = static_cast<float>(kInvalid);
            }
        }
    }

    disparity.width = width;
    disparity.height = height;
    disparity.data.resize(static_cast<std::size_t>(width) * height);
    for (int y = 0; y < height; ++y) {
        const auto* left_row = left_values.ptr<float>(y);
        auto* out = &disparity.data[static_cast<std::size_t>(y) * width];
        std::copy(left_row, left_row + width, out);
    }
    return true;
}

} // namespace sv
