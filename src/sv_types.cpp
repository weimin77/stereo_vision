// Conan::ImportStart
#include "sv_types.hpp"

#include <algorithm>
#include <cmath>
// Conan::ImportEnd

namespace sv {

namespace {

constexpr std::size_t kChannelsGray = 1;
constexpr std::size_t kChannelsColor = 3;
constexpr std::size_t kPercentileCount = 3;
constexpr double kPercentLow = 5.0;
constexpr double kPercentMid = 50.0;
constexpr double kPercentHigh = 95.0;
constexpr double kPercentScale = 100.0;

/// BGR weights of the ITU-R BT.601 luma transform, scaled by 2^8 to stay in integers.
constexpr int kLumaBlue = 29;
constexpr int kLumaGreen = 150;
constexpr int kLumaRed = 77;
constexpr int kLumaShift = 8;

std::size_t pixel_count(int width, int height) {
    if (width <= 0 || height <= 0) {
        return 0;
    }
    return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
}

} // namespace


bool image_valid(const Image& image) {
    if (image.width <= 0 || image.height <= 0) {
        return false;
    }
    const auto channels = static_cast<std::size_t>(image.channels);
    if (channels != kChannelsGray && channels != kChannelsColor) {
        return false;
    }
    return image.data.size() == pixel_count(image.width, image.height) * channels;
}

bool disparity_valid(const DisparityImage& disparity) {
    if (disparity.width <= 0 || disparity.height <= 0) {
        return false;
    }
    return disparity.data.size() == pixel_count(disparity.width, disparity.height);
}

bool depth_valid(const DepthImage& depth) {
    if (depth.width <= 0 || depth.height <= 0) {
        return false;
    }
    return depth.data.size() == pixel_count(depth.width, depth.height);
}

void image_to_gray(const Image& source, Image& gray) {
    if (source.channels == kChannelsGray) {
        gray = source;
        return;
    }

    gray.width = source.width;
    gray.height = source.height;
    gray.channels = static_cast<int>(kChannelsGray);
    gray.data.resize(pixel_count(source.width, source.height));

    const auto round_shift = 1 << (kLumaShift - 1);
    for (std::size_t i = 0; i < gray.data.size(); ++i) {
        const std::size_t base = i * kChannelsColor;
        const int blue = source.data[base];
        const int green = source.data[base + 1];
        const int red = source.data[base + 2];
        const int luma = (kLumaBlue * blue + kLumaGreen * green + kLumaRed * red + round_shift) >> kLumaShift;
        gray.data[i] = static_cast<std::uint8_t>(std::clamp(luma, 0, static_cast<int>(kChannelMax)));
    }
}

void percentiles(const std::vector<float>& values, double (&out)[kPercentileCount]) { // NOLINT
    std::fill(std::begin(out), std::end(out), 0.0);

    std::vector<float> finite;
    finite.reserve(values.size());
    for (const float value : values) {
        if (std::isfinite(value)) {
            finite.push_back(value);
        }
    }
    if (finite.empty()) {
        return;
    }

    std::sort(finite.begin(), finite.end());
    const double last = static_cast<double>(finite.size() - 1);
    const double percentiles_in[3] = {kPercentLow, kPercentMid, kPercentHigh};
    for (std::size_t i = 0; i < kPercentileCount; ++i) {
        const double position = percentiles_in[i] / kPercentScale * last;
        const auto lower = static_cast<std::size_t>(position);
        const auto upper = std::min(lower + 1, finite.size() - 1);
        const double weight = position - static_cast<double>(lower);
        out[i] = static_cast<double>(finite[lower]) * (1.0 - weight) + static_cast<double>(finite[upper]) * weight;
    }
}

} // namespace sv
