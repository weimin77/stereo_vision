// Conan::ImportStart
#include "sv_calibration.hpp"

#include <cmath>
#include <cstddef>

#include <opencv2/core.hpp>
// Conan::ImportEnd

namespace sv {

namespace {

constexpr double kEpsilon = 1e-12;

void set_error(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

/// Read a node as a row-major array of doubles; a stored float matrix is widened.
bool read_matrix(const cv::FileNode& node, double* out, std::size_t expected) {
    if (node.empty() || !node.isMap()) {
        return false;
    }
    cv::Mat matrix;
    node >> matrix;
    if (matrix.empty()) {
        return false;
    }
    cv::Mat wide;
    matrix.convertTo(wide, CV_64F);
    if (wide.total() != expected) {
        return false;
    }
    for (std::size_t i = 0; i < expected; ++i) {
        out[i] = wide.ptr<double>()[i];
    }
    return true;
}

bool read_vector(const cv::FileNode& node, std::vector<double>& out) {
    if (node.empty() || !node.isMap()) {
        return false;
    }
    cv::Mat matrix;
    node >> matrix;
    if (matrix.empty()) {
        return false;
    }
    cv::Mat wide;
    matrix.convertTo(wide, CV_64F);
    out.assign(wide.ptr<double>(), wide.ptr<double>() + wide.total());
    return true;
}

struct CameraSlot {
    std::array<double, kMatrix3Size>* matrix;
    std::vector<double>* distortion;
};

bool open_read(const std::string& path, cv::FileStorage& storage, std::string* error) {
    storage.open(path, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        set_error(error, "cannot open calibration file: " + path);
        return false;
    }
    return true;
}

bool apply_camera(const cv::FileStorage& storage, const CameraSlot& slot, const std::string& matrix_key,
                  const std::string& distortion_key, std::string* error) {
    if (!read_matrix(storage[matrix_key], slot.matrix->data(), kMatrix3Size)) {
        set_error(error, "missing or malformed " + matrix_key + " (expected a 3x3 matrix)");
        return false;
    }
    if (!read_vector(storage[distortion_key], *slot.distortion)) {
        set_error(error, "missing or malformed " + distortion_key + " (expected a coefficient vector)");
        return false;
    }
    return true;
}

} // namespace


bool calibration_load_stereo(const std::string& path, Calibration& calibration, std::string* error) { // NOLINT
    cv::FileStorage storage;
    if (!open_read(path, storage, error)) {
        return false;
    }

    if (!read_matrix(storage["M1"], calibration.m1.data(), kMatrix3Size) ||
        !read_matrix(storage["M2"], calibration.m2.data(), kMatrix3Size)) {
        set_error(error, "calibration needs M1 and M2 as 3x3 matrices: " + path);
        return false;
    }
    if (!read_vector(storage["D1"], calibration.d1) || !read_vector(storage["D2"], calibration.d2)) {
        set_error(error, "calibration needs D1 and D2 coefficient vectors: " + path);
        return false;
    }

    calibration.has_extrinsics = read_matrix(storage["R"], calibration.r.data(), kMatrix3Size) &&
                                 read_matrix(storage["T"], calibration.t.data(), kVector3Size);

    calibration.has_rectification = read_matrix(storage["R1"], calibration.r1.data(), kMatrix3Size) &&
                                    read_matrix(storage["R2"], calibration.r2.data(), kMatrix3Size) &&
                                    read_matrix(storage["P1"], calibration.p1.data(), kMatrix3x4Size) &&
                                    read_matrix(storage["P2"], calibration.p2.data(), kMatrix3x4Size);

    if (calibration.has_rectification) {
        // Q is optional even in a rectification file; a missing Q is rebuilt from P.
        read_matrix(storage["Q"], calibration.q.data(), kMatrix4Size);
    }

    return calibration_valid(calibration, error);
}

bool calibration_override_left(const std::string& path, Calibration& calibration, std::string* error) { // NOLINT
    cv::FileStorage storage;
    if (!open_read(path, storage, error)) {
        return false;
    }
    const CameraSlot slot{&calibration.m1, &calibration.d1};
    return apply_camera(storage, slot, "M1", "D1", error);
}

bool calibration_override_right(const std::string& path, Calibration& calibration, std::string* error) { // NOLINT
    cv::FileStorage storage;
    if (!open_read(path, storage, error)) {
        return false;
    }
    const CameraSlot slot{&calibration.m2, &calibration.d2};
    return apply_camera(storage, slot, "M2", "D2", error);
}

bool calibration_valid(const Calibration& calibration, std::string* error) {
    // Pinhole intrinsics live at (0,0) and (1,1); both must be positive for any projection.
    if (!(calibration.m1[0] > 0.0) || !(calibration.m1[4] > 0.0)) {
        set_error(error, "left camera has a non-positive focal length");
        return false;
    }
    if (!(calibration.m2[0] > 0.0) || !(calibration.m2[4] > 0.0)) {
        set_error(error, "right camera has a non-positive focal length");
        return false;
    }
    if (calibration.has_rectification) {
        if (!(calibration.p1[0] > 0.0) || !(calibration.p2[0] > 0.0)) {
            set_error(error, "stored rectification has a non-positive focal length");
            return false;
        }
    } else if (calibration.has_extrinsics) {
        const double norm = std::sqrt(calibration.t[0] * calibration.t[0] + calibration.t[1] * calibration.t[1] +
                                      calibration.t[2] * calibration.t[2]);
        if (norm < kEpsilon) {
            set_error(error, "calibration baseline is zero");
            return false;
        }
    }
    return true;
}

} // namespace sv
