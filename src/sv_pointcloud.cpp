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

/// A cloud entry counts as real when its coordinates are finite.
bool point_finite(const Point& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

} // namespace


bool pointcloud_from_depth(const DepthImage& depth, const RectifyGeometry& geometry, bool keep_invalid,
                           PointCloud& cloud, std::string* error) {
    if (!depth_valid(depth)) {
        set_error(error, "depth map is empty or inconsistent");
        return false;
    }
    if (!(geometry.fx > kZero) || !(geometry.fy > kZero)) {
        set_error(error, "geometry has a non-positive focal length");
        return false;
    }

    cloud.points.clear();
    cloud.points.reserve(keep_invalid ? depth.data.size() : depth.data.size() / 2);

    for (int y = 0; y < depth.height; ++y) {
        const auto* depth_row = &depth.data[static_cast<std::size_t>(y) * depth.width];
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
        << "FIELDS x y z\n"
        << "SIZE 4 4 4\n"
        << "TYPE F F F\n"
        << "COUNT 1 1 1\n"
        << "WIDTH " << count << '\n'
        << "HEIGHT 1\n"
        << "VIEWPOINT 0 0 0 1 0 0 0\n"
        << "POINTS " << count << '\n'
        << "DATA " << (binary ? "binary" : "ascii") << '\n';

    if (binary) {
        for (const Point& point : cloud.points) {
            const float values[3] = {point.x, point.y, point.z};
            out.write(reinterpret_cast<const char*>(values), sizeof(values));
        }
    } else {
        out.setf(std::ios::fixed, std::ios::floatfield);
        out.precision(6);
        for (const Point& point : cloud.points) {
            if (point_finite(point)) {
                out << point.x << ' ' << point.y << ' ' << point.z << '\n';
            } else {
                out << "nan nan nan\n";
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
        << "property float z\n"
        << "end_header\n";

    if (binary) {
        for (const Point& point : cloud.points) {
            const float xyz[3] = {point.x, point.y, point.z};
            out.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
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
