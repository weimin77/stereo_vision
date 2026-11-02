// Conan::ImportStart
#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "sv_types.hpp"
// Conan::ImportEnd

namespace sv {

/// Number of elements in a 3x4 projection matrix stored row-major.
inline constexpr std::size_t kMatrix3x4Size = 12;

/// Number of translation elements.
inline constexpr std::size_t kVector3Size = 3;

/**
 * @brief [en] Stereo calibration: intrinsics, distortion, relative pose and optional rectification
 * @brief [zh] 双目标定：内参、畸变、相对位姿与可选的校正变换
 *
 * A stereo.yml written by cv::stereoCalibrate fills M1/D1/M2/D2/R/T (plus E and F, which
 * this struct ignores). A rectification.yml additionally carries R1/R2/P1/P2/Q. Mono
 * intrinsics files carry only M/D for one camera and override the stereo file.
 *
 * @since 1.0
 * @exporter
 */
struct Calibration {
    std::array<double, kMatrix3Size> m1{};
    std::array<double, kMatrix3Size> m2{};
    std::vector<double> d1;
    std::vector<double> d2;
    std::array<double, kMatrix3Size> r{};
    std::array<double, kVector3Size> t{};
    std::array<double, kMatrix3Size> r1{};
    std::array<double, kMatrix3Size> r2{};
    std::array<double, kMatrix3x4Size> p1{};
    std::array<double, kMatrix3x4Size> p2{};
    std::array<double, kMatrix4Size> q{};
    bool has_extrinsics = false;
    bool has_rectification = false;
};

/**
 * @brief [en] Load the mandatory intrinsics and optional extrinsics from a stereo calibration
 * @brief [zh] 从双目标定文件读取必需的内参与可选的相对位姿
 * @param path stereo.yml path / stereo.yml 路径
 * @param calibration receives the values / 接收读取结果
 * @param error optional message, written on failure / 失败时写入原因
 * @return true when M1, D1, M2 and D2 are present and well formed
 * @since 1.0
 * @exporter
 */
bool calibration_load_stereo(const std::string& path, Calibration& calibration,
                             std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

/**
 * @brief [en] Override the left camera intrinsics from a single-camera calibration file
 * @brief [zh] 用左目单目标定文件覆盖左相机内参
 * @param path mono intrinsics path / 单目内参文件路径
 * @param calibration calibration to update / 待更新的标定
 * @param error optional message, written on failure / 失败时写入原因
 * @return true when M1 and D1 were found and applied
 * @since 1.0
 * @exporter
 */
bool calibration_override_left(const std::string& path, Calibration& calibration,
                               std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

/**
 * @brief [en] Override the right camera intrinsics from a single-camera calibration file
 * @brief [zh] 用右目单目标定文件覆盖右相机内参
 * @param path mono intrinsics path / 单目内参文件路径
 * @param calibration calibration to update / 待更新的标定
 * @param error optional message, written on failure / 失败时写入原因
 * @return true when M2 and D2 were found and applied
 * @since 1.0
 * @exporter
 */
bool calibration_override_right(const std::string& path, Calibration& calibration,
                                std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

/**
 * @brief [en] Check that the calibration can drive a rectification or a depth conversion
 * @brief [zh] 检查标定是否足以支撑校正或深度换算
 * @param calibration calibration to check / 待检查标定
 * @param error optional message, written on failure / 失败时写入原因
 * @return true when focal lengths are positive and a baseline is available
 * @since 1.0
 * @exporter
 */
bool calibration_valid(const Calibration& calibration, std::string* error);

} // namespace sv
