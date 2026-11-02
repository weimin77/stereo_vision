#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "sv_calibration.hpp"
#include "sv_types.hpp"

namespace synthetic {

constexpr double kTestFocal = 500.0;
constexpr double kTestBaseline = 0.1;
constexpr int kTestCx = 320;
constexpr int kTestCy = 240;
constexpr int kTestWidth = 640;
constexpr int kTestHeight = 480;
constexpr int kTextureBlock = 4;
constexpr std::uint32_t kLcgMultiplier = 1664525U;
constexpr std::uint32_t kLcgIncrement = 1013904223U;
constexpr int kLcgShift = 24;

/// A fronto-parallel pinhole rig with zero distortion, so rectification is the identity.
inline sv::Calibration pinhole_rig() {
    sv::Calibration calibration;
    const double intrinsics[sv::kMatrix3Size] = {kTestFocal, 0.0, kTestCx, 0.0, kTestFocal, kTestCy, 0.0, 0.0, 1.0};
    for (std::size_t i = 0; i < sv::kMatrix3Size; ++i) {
        calibration.m1[i] = intrinsics[i];
        calibration.m2[i] = intrinsics[i];
        calibration.r[i] = (i % 4 == 0) ? 1.0 : 0.0;
    }
    calibration.d1.assign(5, 0.0);
    calibration.d2.assign(5, 0.0);
    calibration.t = {-kTestBaseline, 0.0, 0.0};
    calibration.has_extrinsics = true;
    return calibration;
}

/// Same rig but with the second camera on the +x side, which the pipeline must refuse.
inline sv::Calibration mirrored_rig() {
    sv::Calibration calibration = pinhole_rig();
    calibration.t = {kTestBaseline, 0.0, 0.0};
    return calibration;
}

/// Coarse random texture: block matching needs structure, not white noise.
inline sv::Image blocky_texture(int width, int height, std::uint32_t seed) {
    sv::Image image;
    image.width = width;
    image.height = height;
    image.channels = 1;
    image.data.resize(static_cast<std::size_t>(width) * height);

    const int coarse_width = (width / kTextureBlock) + 1;
    const int coarse_height = (height / kTextureBlock) + 1;
    std::vector<std::uint8_t> coarse(static_cast<std::size_t>(coarse_width) * coarse_height);

    std::uint32_t state = seed;
    for (auto& value : coarse) {
        state = (state * kLcgMultiplier) + kLcgIncrement;
        value = static_cast<std::uint8_t>(state >> kLcgShift);
    }

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int cx = x / kTextureBlock;
            const int cy = y / kTextureBlock;
            image.data[(static_cast<std::size_t>(y) * width) + x] =
                coarse[(static_cast<std::size_t>(cy) * coarse_width) + cx];
        }
    }
    return image;
}

/// right(x) = left(x + shift), i.e. the same scene seen `shift` pixels further left.
inline sv::Image shift_horizontal(const sv::Image& source, int shift) {
    sv::Image shifted;
    shifted.width = source.width;
    shifted.height = source.height;
    shifted.channels = source.channels;
    shifted.data.assign(source.data.size(), 0);

    for (int y = 0; y < source.height; ++y) {
        for (int x = 0; x < source.width; ++x) {
            const int source_x = x + shift;
            if (source_x >= source.width) {
                continue;
            }
            shifted.data[(static_cast<std::size_t>(y) * source.width) + x] =
                source.data[(static_cast<std::size_t>(y) * source.width) + source_x];
        }
    }
    return shifted;
}

/// Median of the finite entries of a disparity or depth map.
inline double median_of_finite(const std::vector<float>& values) {
    std::vector<float> finite;
    finite.reserve(values.size());
    for (const float value : values) {
        if (value == value) {
            finite.push_back(value);
        }
    }
    if (finite.empty()) {
        return 0.0;
    }
    const std::size_t mid = finite.size() / 2;
    std::nth_element(finite.begin(), finite.begin() + static_cast<std::ptrdiff_t>(mid), finite.end());
    return finite[mid];
}

}  // namespace synthetic
