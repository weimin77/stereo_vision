// Conan::ImportStart
#include "sv_image_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <system_error>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
// Conan::ImportEnd

namespace sv {

namespace {

namespace fs = std::filesystem;

constexpr int kChannelsGray = 1;
constexpr int kChannelsBgr = 3;
constexpr int kChannelsBgra = 4;

std::string lower_extension(const std::string& path) {
    std::string ext = fs::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

bool is_image_extension(const std::string& ext) {
    return ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp";
}

bool is_png(const std::string& ext) { return ext == ".png"; }

bool is_tiff(const std::string& ext) { return ext == ".tif" || ext == ".tiff"; }

void set_error(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

/// imwrite picks the encoder from the extension, so the temporary keeps it and only the stem changes.
fs::path temporary_path(const fs::path& target) {
    fs::path temp = target;
    temp.replace_filename(target.stem().string() + ".tmp" + target.extension().string());
    return temp;
}

bool write_atomically(const std::string& path, const cv::Mat& mat, std::string* error) {
    const fs::path target(path);
    std::error_code code;
    if (target.has_parent_path()) {
        const fs::path parent = target.parent_path();
        if (!fs::exists(parent, code) || !fs::is_directory(parent, code)) {
            set_error(error, "output directory does not exist: " + parent.string());
            return false;
        }
    }

    const fs::path temp = temporary_path(target);
    if (!cv::imwrite(temp.string(), mat)) {
        set_error(error, "cannot write image: " + path);
        return false;
    }
    fs::rename(temp, target, code);
    if (code) {
        fs::remove(temp, code);
        set_error(error, "cannot replace " + path + ": " + code.message());
        return false;
    }
    return true;
}

cv::Mat to_cv(const Image& image) {
    const int type = image.channels == kChannelsGray ? CV_8UC1 : CV_8UC3;
    cv::Mat mat(image.height, image.width, type);
    std::copy(image.data.begin(), image.data.end(), mat.ptr<std::uint8_t>());
    return mat;
}

constexpr double kLevelMax = static_cast<double>(kColorLevels - 1);
constexpr double kUnitSpan = 1.0;
constexpr double kSpanPadding = 0.5;
constexpr std::uint8_t kMaskSet = 1;

/// Every finite sample of a flat map lands on the same level, so stretching it to the middle still shows it.
struct Range {
    double low = 0.0;
    double high = kUnitSpan;
};

Range robust_range(const double (&anchor)[3]) { // NOLINT(cppcoreguidelines-avoid-c-arrays)
    Range range{anchor[0], anchor[2]};
    if (!(range.high > range.low)) {
        range.low = anchor[1] - kSpanPadding;
        range.high = range.low + kUnitSpan;
    }
    return range;
}

/// Thermal ramp for looking at, not for measuring: the 16-bit/f32 writers stay the measurement path.
cv::Mat colorize_map(const std::vector<float>& values, int width, int height, bool positive_only, bool invert) {
    double anchor[3] = {0.0, 0.0, 0.0}; // NOLINT(cppcoreguidelines-avoid-c-arrays)
    percentiles(values, anchor);
    const Range range = robust_range(anchor);

    cv::Mat levels(height, width, CV_8UC1, cv::Scalar(0));
    cv::Mat valid(height, width, CV_8UC1, cv::Scalar(0));
    auto* level = levels.ptr<std::uint8_t>();
    auto* mask = valid.ptr<std::uint8_t>();
    for (std::size_t i = 0; i < values.size(); ++i) {
        const float value = values[i];
        if (!std::isfinite(value) || (positive_only && value <= 0.0F)) {
            continue;
        }
        double fraction = (static_cast<double>(value) - range.low) / (range.high - range.low);
        fraction = std::clamp(fraction, 0.0, kUnitSpan);
        if (invert) {
            fraction = kUnitSpan - fraction;
        }
        level[i] = static_cast<std::uint8_t>(std::lround(fraction * kLevelMax));
        mask[i] = kMaskSet;
    }

    cv::Mat color;
    cv::applyColorMap(levels, color, cv::COLORMAP_TURBO);
    // Black marks "no measurement" and stays distinct from the darkest ramp entry.
    color.setTo(cv::Scalar::all(0), valid == 0);
    return color;
}

bool unsupported_colorize_path(const std::string& path, std::string* error) {
    const std::string ext = lower_extension(path);
    if (is_tiff(ext)) {
        set_error(error, "colorize writes 8-bit colour, so it cannot target a float .tif: " + path);
        return true;
    }
    if (!is_image_extension(ext)) {
        set_error(error, "colorized output accepts .png, .jpg, .jpeg or .bmp: " + path);
        return true;
    }
    return false;
}

} // namespace


bool image_path_supported(const std::string& path) { return is_image_extension(lower_extension(path)); }

bool image_load(const std::string& path, Image& image, std::string* error) { // NOLINT
    if (!image_path_supported(path)) {
        set_error(error, "unsupported image extension (expected jpg, jpeg, png or bmp): " + path);
        return false;
    }

    cv::Mat raw = cv::imread(path, cv::IMREAD_UNCHANGED);
    if (raw.empty()) {
        set_error(error, "cannot decode image: " + path);
        return false;
    }

    cv::Mat bgr;
    if (raw.channels() == kChannelsBgra) {
        cv::cvtColor(raw, bgr, cv::COLOR_BGRA2BGR);
    } else if (raw.channels() == kChannelsBgr || raw.channels() == kChannelsGray) {
        bgr = raw;
    } else {
        set_error(error, "unsupported channel count " + std::to_string(raw.channels()) + " in " + path);
        return false;
    }

    if (bgr.depth() != CV_8U) {
        cv::Mat narrowed;
        bgr.convertTo(narrowed, CV_MAKETYPE(CV_8U, bgr.channels()));
        bgr = narrowed;
    }

    image.width = bgr.cols;
    image.height = bgr.rows;
    image.channels = bgr.channels();
    image.data.assign(bgr.data, bgr.data + (bgr.total() * static_cast<std::size_t>(bgr.channels())));
    return true;
}

bool image_save_png(const std::string& path, const Image& image, std::string* error) { // NOLINT
    if (!image_valid(image)) {
        set_error(error, "image is empty or inconsistent");
        return false;
    }
    if (!is_png(lower_extension(path))) {
        set_error(error, "image_save_png accepts only .png: " + path);
        return false;
    }
    return write_atomically(path, to_cv(image), error);
}

bool disparity_save(const std::string& path, const DisparityImage& disparity, std::string* error, // NOLINT
                    bool colorize) {
    if (!disparity_valid(disparity)) {
        set_error(error, "disparity map is empty or inconsistent");
        return false;
    }

    if (colorize) {
        if (unsupported_colorize_path(path, error)) {
            return false;
        }
        // Nearer surfaces are hotter: a larger disparity means a shorter distance.
        return write_atomically(path, colorize_map(disparity.data, disparity.width, disparity.height, false, false),
                                error);
    }

    const std::string ext = lower_extension(path);
    if (is_tiff(ext)) {
        cv::Mat mat(disparity.height, disparity.width, CV_32FC1);
        std::copy(disparity.data.begin(), disparity.data.end(), mat.ptr<float>());
        return write_atomically(path, mat, error);
    }
    if (!is_png(ext)) {
        set_error(error, "disparity output accepts .png, .tif or .tiff: " + path);
        return false;
    }

    cv::Mat mat(disparity.height, disparity.width, CV_16UC1);
    auto* row = mat.ptr<std::uint16_t>();
    for (std::size_t i = 0; i < disparity.data.size(); ++i) {
        const float value = disparity.data[i];
        if (!std::isfinite(value)) {
            row[i] = 0;
            continue;
        }
        const double scaled = std::round(static_cast<double>(value) * kDisparityPngScale);
        const double limited = std::clamp(scaled, 0.0, static_cast<double>(kSample16Max));
        row[i] = static_cast<std::uint16_t>(limited);
    }
    return write_atomically(path, mat, error);
}

bool depth_save(const std::string& path, const DepthImage& depth, std::string* error, bool colorize) { // NOLINT
    if (!depth_valid(depth)) {
        set_error(error, "depth map is empty or inconsistent");
        return false;
    }

    if (colorize) {
        if (unsupported_colorize_path(path, error)) {
            return false;
        }
        // Depth grows away from the camera, so the ramp is inverted: near stays hot, matching the disparity map.
        return write_atomically(path, colorize_map(depth.data, depth.width, depth.height, true, true), error);
    }

    const std::string ext = lower_extension(path);
    if (is_tiff(ext)) {
        cv::Mat mat(depth.height, depth.width, CV_32FC1);
        std::copy(depth.data.begin(), depth.data.end(), mat.ptr<float>());
        return write_atomically(path, mat, error);
    }
    if (!is_png(ext)) {
        set_error(error, "depth output accepts .png, .tif or .tiff: " + path);
        return false;
    }

    cv::Mat mat(depth.height, depth.width, CV_16UC1);
    auto* row = mat.ptr<std::uint16_t>();
    for (std::size_t i = 0; i < depth.data.size(); ++i) {
        const float value = depth.data[i];
        if (!std::isfinite(value) || value <= 0.0F) {
            row[i] = 0;
            continue;
        }
        const double scaled = std::round(static_cast<double>(value) * kDepthPngScale);
        const double limited = std::clamp(scaled, 0.0, static_cast<double>(kSample16Max));
        row[i] = static_cast<std::uint16_t>(limited);
    }
    return write_atomically(path, mat, error);
}

} // namespace sv
