// Conan::ImportStart
#include "sv_depth.hpp"

#include <cmath>
#include <limits>
// Conan::ImportEnd

namespace sv {

namespace {

constexpr double kInvalid = std::numeric_limits<double>::quiet_NaN();
constexpr double kZero = 0.0;

void set_error(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

} // namespace


bool depth_from_disparity(const DisparityImage& disparity, const RectifyGeometry& geometry, double min_m, double max_m,
                          DepthImage& depth, std::string* error) { // NOLINT
    if (!disparity_valid(disparity)) {
        set_error(error, "disparity map is empty or inconsistent");
        return false;
    }
    if (!(geometry.fx > 0.0) || !(geometry.focal_baseline > 0.0)) {
        set_error(error, "geometry has a non-positive focal length or focal baseline");
        return false;
    }
    const bool clamp = min_m != kZero || max_m != kZero;
    if (clamp && (!(min_m > kZero) || !(max_m > min_m))) {
        set_error(error, "depth.min_m and depth.max_m must satisfy 0 < min_m < max_m, or both be 0");
        return false;
    }

    depth.width = disparity.width;
    depth.height = disparity.height;
    depth.data.resize(disparity.data.size());

    const double numerator = geometry.focal_baseline;
    for (std::size_t i = 0; i < disparity.data.size(); ++i) {
        const double value = static_cast<double>(disparity.data[i]) - geometry.doffs;
        if (!(value > kZero)) {
            depth.data[i] = static_cast<float>(kInvalid);
            continue;
        }
        const double metres = numerator / value;
        if (!std::isfinite(metres) || (clamp && (metres < min_m || metres > max_m))) {
            depth.data[i] = static_cast<float>(kInvalid);
            continue;
        }
        depth.data[i] = static_cast<float>(metres);
    }
    return true;
}

} // namespace sv
