// Conan::ImportStart
#pragma once

#include <string>

#include "sv_types.hpp"
// Conan::ImportEnd

namespace sv {

/**
 * @brief [en] Convert a disparity map to depth in the rectified left camera frame
 * @brief [zh] 把视差图转换为校正后左相机坐标系下的深度
 *
 * The relation is Z = fx * |Tx| / (d - doffs), not fx * B / d: the principal points of the
 * two rectified views generally differ, and ignoring doffs shifts the whole cloud sideways.
 *
 * @param disparity sub-pixel disparity in pixels / 亚像素视差，单位像素
 * @param geometry rectified geometry / 校正后的几何参数
 * @param min_m lower depth clamp in metres, 0 disables the clamp / 深度下限，0 表示不裁剪
 * @param max_m upper depth clamp in metres, 0 disables the clamp / 深度上限，0 表示不裁剪
 * @param depth receives the map in metres / 接收深度图，单位米
 * @param error optional message, written on failure / 失败时写入原因
 * @return true on success
 * @since 1.0
 * @exporter
 */
bool depth_from_disparity(const DisparityImage& disparity, const RectifyGeometry& geometry, double min_m, double max_m,
                          DepthImage& depth, std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

} // namespace sv
