// Conan::ImportStart
#include "sv_pointcloud.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <system_error>
#include <vector>
// Conan::ImportEnd

namespace sv {

namespace {

namespace fs = std::filesystem;

constexpr double kZero = 0.0;
constexpr int kChannelsGray = 1;
constexpr int kChannelsColor = 3;
constexpr int kByteCount = 4;
constexpr int kRgbRedShift = 16;
constexpr int kRgbGreenShift = 8;
constexpr float kInvalidFloat = std::numeric_limits<float>::quiet_NaN();

void set_error(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

std::string lower_extension(const std::string& path) {
    std::string ext = fs::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

bool check_target(const std::string& path, std::string* error) {
    const fs::path target(path);
    std::error_code code;
    if (target.has_parent_path()) {
        const fs::path parent = target.parent_path();
        if (!fs::exists(parent, code) || !fs::is_directory(parent, code)) {
            set_error(error, "output directory does not exist: " + parent.string());
            return false;
        }
    }
    return true;
}

/// A cloud entry counts as real when its coordinates are finite and non-zero depth.
bool point_finite(const Point& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

/// PCD stores the colour as the bit pattern of a float, so ascii and binary agree.
float pack_rgb(const Point& point) {
    const std::uint32_t packed = (static_cast<std::uint32_t>(point.r) << kRgbRedShift) |
                                 (static_cast<std::uint32_t>(point.g) << kRgbGreenShift) |
                                 static_cast<std::uint32_t>(point.b);
    float value = 0.0F;
    std::memcpy(&value, &packed, sizeof(value));
    return value;
}

std::uint32_t pack_rgb_bits(const Point& point) {
    return (static_cast<std::uint32_t>(point.r) << kRgbRedShift) |
           (static_cast<std::uint32_t>(point.g) << kRgbGreenShift) | static_cast<std::uint32_t>(point.b);
}

} // namespace


bool pointcloud_from_depth(const DepthImage& depth, const Image& color, const RectifyGeometry& geometry,
                           bool keep_invalid, PointCloud& cloud, std::string* error) { // NOLINT
    if (!depth_valid(depth)) {
        set_error(error, "depth map is empty or inconsistent");
        return false;
    }
    if (!(geometry.fx > kZero) || !(geometry.fy > kZero)) {
        set_error(error, "geometry has a non-positive focal length");
        return false;
    }
    if (!color.data.empty()) {
        if (!image_valid(color)) {
            set_error(error, "colour image is empty or inconsistent");
            return false;
        }
        if (color.width != depth.width || color.height != depth.height) {
            set_error(error, "colour image must match the depth map size");
            return false;
        }
        if (color.channels != kChannelsColor && color.channels != kChannelsGray) {
            set_error(error, "colour image must have 1 or 3 channels");
            return false;
        }
    }

    // A grayscale source is replicated onto r/g/b: the cloud is still "colored", just grey,
    // so one reader path handles both cases.
    const bool colored = !color.data.empty();
    const int color_step = colored ? color.channels : 0;
    cloud.colored = colored;
    cloud.points.clear();
    cloud.points.reserve(keep_invalid ? depth.data.size() : depth.data.size() / 2);

    for (int y = 0; y < depth.height; ++y) {
        const auto* depth_row = &depth.data[static_cast<std::size_t>(y) * depth.width];
        const std::uint8_t* color_row =
            colored ? &color.data[static_cast<std::size_t>(y) * depth.width * static_cast<std::size_t>(color_step)]
                    : nullptr;
        for (int x = 0; x < depth.width; ++x) {
            const float z = depth_row[x];
            const bool usable = std::isfinite(z) && z > kZero;
            if (!usable && !keep_invalid) {
                continue;
            }
            Point point;
            if (usable) {
                point.x = static_cast<float>((static_cast<double>(x) - geometry.cx) * z / geometry.fx);
                point.y = static_cast<float>((static_cast<double>(y) - geometry.cy) * z / geometry.fy);
                point.z = z;
            } else {
                point.x = kInvalidFloat;
                point.y = kInvalidFloat;
                point.z = kInvalidFloat;
            }
            if (colored) {
                const std::uint8_t* pixel =
                    color_row + (static_cast<std::size_t>(x) * static_cast<std::size_t>(color_step));
                if (color_step == kChannelsColor) {
                    point.b = pixel[0];
                    point.g = pixel[1];
                    point.r = pixel[2];
                } else {
                    point.b = pixel[0];
                    point.g = pixel[0];
                    point.r = pixel[0];
                }
            }
            cloud.points.push_back(point);
        }
    }
    return true;
}

bool pointcloud_save_pcd(const std::string& path, const PointCloud& cloud, bool binary, std::string* error) { // NOLINT
    if (lower_extension(path) != ".pcd") {
        set_error(error, "pointcloud_save_pcd accepts only .pcd: " + path);
        return false;
    }
    if (!check_target(path, error)) {
        return false;
    }

    const fs::path target(path);
    fs::path temp = target;
    temp += ".tmp";

    std::ofstream out(temp, std::ios::binary);
    if (!out) {
        set_error(error, "cannot open for writing: " + temp.string());
        return false;
    }

    const std::size_t count = cloud.points.size();
    out << "# .PCD v0.7 - Point Cloud Data file format\n"
        << "VERSION 0.7\n"
        << "FIELDS x y z rgb\n"
        << "SIZE 4 4 4 4\n"
        << "TYPE F F F F\n"
        << "COUNT 1 1 1 1\n"
        << "WIDTH " << count << '\n'
        << "HEIGHT 1\n"
        << "VIEWPOINT 0 0 0 1 0 0 0\n"
        << "POINTS " << count << '\n'
        << "DATA " << (binary ? "binary" : "ascii") << '\n';

    if (binary) {
        for (const Point& point : cloud.points) {
            const std::uint32_t rgb = pack_rgb_bits(point);
            float packed = 0.0F;
            std::memcpy(&packed, &rgb, sizeof(packed));
            const float values[kByteCount] = {point.x, point.y, point.z, packed};
            out.write(reinterpret_cast<const char*>(values), sizeof(values));
        }
    } else {
        out.setf(std::ios::fixed, std::ios::floatfield);
        out.precision(6);
        for (const Point& point : cloud.points) {
            if (point_finite(point)) {
                out << point.x << ' ' << point.y << ' ' << point.z << ' ' << static_cast<double>(pack_rgb(point))
                    << '\n';
            } else {
                out << "nan nan nan 0\n";
            }
        }
    }
    out.close();
    if (!out) {
        set_error(error, "write failed: " + temp.string());
        fs::remove(temp);
        return false;
    }

    std::error_code code;
    fs::rename(temp, target, code);
    if (code) {
        fs::remove(temp, code);
        set_error(error, "cannot replace " + path);
        return false;
    }
    return true;
}

bool pointcloud_save_ply(const std::string& path, const PointCloud& cloud, bool binary, std::string* error) { // NOLINT
    if (lower_extension(path) != ".ply") {
        set_error(error, "pointcloud_save_ply accepts only .ply: " + path);
        return false;
    }
    if (!check_target(path, error)) {
        return false;
    }

    const fs::path target(path);
    fs::path temp = target;
    temp += ".tmp";

    std::ofstream out(temp, std::ios::binary);
    if (!out) {
        set_error(error, "cannot open for writing: " + temp.string());
        return false;
    }

    const std::size_t count = cloud.points.size();
    out << "ply\n"
        << "format " << (binary ? "binary_little_endian" : "ascii") << " 1.0\n"
        << "comment generated by stereo_vision\n"
        << "element vertex " << count << '\n'
        << "property float x\n"
        << "property float y\n"
        << "property float z\n";
    if (cloud.colored) {
        out << "property uchar red\n"
            << "property uchar green\n"
            << "property uchar blue\n";
    }
    out << "end_header\n";

    if (binary) {
        for (const Point& point : cloud.points) {
            const float xyz[3] = {point.x, point.y, point.z};
            out.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
            if (cloud.colored) {
                const std::uint8_t rgb[3] = {point.r, point.g, point.b};
                out.write(reinterpret_cast<const char*>(rgb), sizeof(rgb));
            }
        }
    } else {
        out.setf(std::ios::fixed, std::ios::floatfield);
        out.precision(6);
        for (const Point& point : cloud.points) {
            if (point_finite(point)) {
                out << point.x << ' ' << point.y << ' ' << point.z;
            } else {
                out << "nan nan nan";
            }
            if (cloud.colored) {
                out << ' ' << static_cast<int>(point.r) << ' ' << static_cast<int>(point.g) << ' '
                    << static_cast<int>(point.b);
            }
            out << '\n';
        }
    }
    out.close();
    if (!out) {
        set_error(error, "write failed: " + temp.string());
        fs::remove(temp);
        return false;
    }

    std::error_code code;
    fs::rename(temp, target, code);
    if (code) {
        fs::remove(temp, code);
        set_error(error, "cannot replace " + path);
        return false;
    }
    return true;
}

bool pointcloud_save(const std::string& path, const PointCloud& cloud, bool binary, std::string* error) { // NOLINT
    const std::string ext = lower_extension(path);
    if (ext == ".pcd") {
        return pointcloud_save_pcd(path, cloud, binary, error);
    }
    if (ext == ".ply") {
        return pointcloud_save_ply(path, cloud, binary, error);
    }
    set_error(error, "point cloud output accepts .pcd or .ply: " + path);
    return false;
}

} // namespace sv
