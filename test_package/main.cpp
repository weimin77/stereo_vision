#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "sv_config.hpp"
#include "sv_pipeline.hpp"
#include "sv_types.hpp"

namespace {

constexpr int kWidth = 320;
constexpr int kHeight = 240;
constexpr int kShift = 20;
constexpr int kNumDisparities = 64;
constexpr std::uint32_t kSeed = 20261102U;
constexpr std::uint32_t kLcgMultiplier = 1664525U;
constexpr std::uint32_t kLcgIncrement = 1013904223U;
constexpr int kLcgShift = 24;
constexpr int kTextureBlock = 4;

sv::Image texture(int width, int height) {
    sv::Image image;
    image.width = width;
    image.height = height;
    image.channels = 1;
    image.data.resize(static_cast<std::size_t>(width) * height);

    const int coarse_width = (width / kTextureBlock) + 1;
    const int coarse_height = (height / kTextureBlock) + 1;
    std::vector<std::uint8_t> coarse(static_cast<std::size_t>(coarse_width) * coarse_height);
    std::uint32_t state = kSeed;
    for (auto& value : coarse) {
        state = (state * kLcgMultiplier) + kLcgIncrement;
        value = static_cast<std::uint8_t>(state >> kLcgShift);
    }
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            image.data[(static_cast<std::size_t>(y) * width) + x] =
                coarse[(static_cast<std::size_t>(y / kTextureBlock) * coarse_width) + (x / kTextureBlock)];
        }
    }
    return image;
}

sv::Calibration rig() {
    sv::Calibration calibration;
    const double focal = 500.0;
    const double intrinsics[sv::kMatrix3Size] = {focal, 0.0, 160.0, 0.0, focal, 120.0, 0.0, 0.0, 1.0};
    for (std::size_t i = 0; i < sv::kMatrix3Size; ++i) {
        calibration.m1[i] = intrinsics[i];
        calibration.m2[i] = intrinsics[i];
        calibration.r[i] = (i % 4 == 0) ? 1.0 : 0.0;
    }
    calibration.d1.assign(5, 0.0);
    calibration.d2.assign(5, 0.0);
    calibration.t = {-0.1, 0.0, 0.0};
    calibration.has_extrinsics = true;
    return calibration;
}

}  // namespace

int main() {
    sv::Config config = sv::config_default();
    config.matcher.num_disparities = kNumDisparities;
    config.matcher.mode = sv::SgbmMode::Sgbm;
    config.rectify.crop_to_common_roi = false;

    const sv::Image left = texture(kWidth, kHeight);
    sv::Image right = left;
    std::fill(right.data.begin(), right.data.end(), 0);
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x + kShift < kWidth; ++x) {
            right.data[(static_cast<std::size_t>(y) * kWidth) + x] =
                left.data[(static_cast<std::size_t>(y) * kWidth) + x + kShift];
        }
    }

    if (!image_valid(left) || !image_valid(right)) {
        std::cerr << "smoke: synthetic pair is invalid\n";
        return 1;
    }

    sv::Pipeline pipeline{config};
    if (!pipeline.set_calibration(rig())) {
        std::cerr << "smoke: " << pipeline.last_error() << '\n';
        return 1;
    }
    if (!pipeline.set_input_image(left, right)) {
        std::cerr << "smoke: " << pipeline.last_error() << '\n';
        return 1;
    }
    if (!pipeline.run()) {
        std::cerr << "smoke: " << pipeline.last_error() << '\n';
        return 1;
    }

    const sv::RunReport& report = pipeline.report();
    std::cout << "rectified " << report.rectified_width << "x" << report.rectified_height << '\n'
              << "window    " << report.min_disparity << " + " << report.num_disparities << '\n'
              << "valid     " << report.valid_ratio << '\n'
              << "disparity p05/p50/p95 " << report.disparity_p05 << " / " << report.disparity_p50 << " / "
              << report.disparity_p95 << '\n'
              << "depth     p05/p50/p95 " << report.depth_p05 << " / " << report.depth_p50 << " / "
              << report.depth_p95 << '\n'
              << "cloud     " << pipeline.get_pointcloud().points.size() << " points\n";

    if (!(report.valid_ratio > 0.0)) {
        std::cerr << "smoke: no valid disparity\n";
        return 1;
    }
    return 0;
}
