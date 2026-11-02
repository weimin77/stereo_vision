// Conan::ImportStart
#pragma once

#include <string>

#include "sv_types.hpp"
// Conan::ImportEnd

namespace sv {

/**
 * @brief [en] Back-project a depth map into the rectified left camera frame
 * @brief [zh] 把深度图反投影到校正后的左相机坐标系
 * @param depth depth map in metres / 深度图，单位米
 * @param geometry rectified geometry / 校正后的几何参数
 * @param keep_invalid keep one entry per pixel, marking invalid depth as NaN
 * @param cloud receives the points / 接收点云
 * @param error optional message, written on failure / 失败时写入原因
 * @return true on success
 * @since 1.0
 * @exporter
 */
bool pointcloud_from_depth(const DepthImage& depth, const RectifyGeometry& geometry, bool keep_invalid,
                           PointCloud& cloud, std::string* error);

/**
 * @brief [en] Write a point cloud as PCD, choosing ascii or binary from the encoding flag
 * @brief [zh] 把点云写为 PCD，按 encoding 选择 ascii 或 binary
 * @param path output path, .pcd only / 输出路径，仅支持 .pcd
 * @param cloud points to write / 待写出的点
 * @param binary true for binary, false for ascii / true 为 binary，false 为 ascii
 * @param error optional message, written on failure / 失败时写入原因
 * @return true on success
 * @since 1.0
 * @exporter
 */
bool pointcloud_save_pcd(const std::string& path, const PointCloud& cloud, bool binary,
                         std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

/**
 * @brief [en] Write a point cloud as PLY, choosing ascii or binary from the encoding flag
 * @brief [zh] 把点云写为 PLY，按 encoding 选择 ascii 或 binary
 * @param path output path, .ply only / 输出路径，仅支持 .ply
 * @param cloud points to write / 待写出的点
 * @param binary true for binary_little_endian, false for ascii / true 为 binary_little_endian，false 为 ascii
 * @param error optional message, written on failure / 失败时写入原因
 * @return true on success
 * @since 1.0
 * @exporter
 */
bool pointcloud_save_ply(const std::string& path, const PointCloud& cloud, bool binary,
                         std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

/**
 * @brief [en] Write a point cloud, picking the format from the path extension
 * @brief [zh] 按路径扩展名选择格式写出点云
 * @param path output path, .pcd or .ply / 输出路径，.pcd 或 .ply
 * @param cloud points to write / 待写出的点
 * @param binary true for a binary encoding / true 表示二进制编码
 * @param error optional message, written on failure / 失败时写入原因
 * @return true on success
 * @since 1.0
 * @exporter
 */
bool pointcloud_save(const std::string& path, const PointCloud& cloud, bool binary,
                     std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

} // namespace sv
