// Conan::ImportStart
#pragma once

#include <string>

#include "sv_types.hpp"
// Conan::ImportEnd

namespace sv {

/// Fixed-point scale of a 16-bit disparity PNG: one pixel of disparity is 16 counts.
inline constexpr double kDisparityPngScale = 16.0;

/// Scale of a 16-bit depth PNG: one metre is 1000 counts.
inline constexpr double kDepthPngScale = 1000.0;

/// Largest value a 16-bit sample can hold.
inline constexpr int kSample16Max = 65535;

/**
 * @brief [en] Whether the path ends in jpg, jpeg, png or bmp
 * @brief [zh] 判断路径扩展名是否为 jpg、jpeg、png 或 bmp
 * @param path file path / 文件路径
 * @return true when the extension is one of the supported input formats
 * @since 1.0
 * @exporter
 */
bool image_path_supported(const std::string& path);

/**
 * @brief [en] Load an 8-bit image, keeping its colour channel count
 * @brief [zh] 读取 8 位图像，保留原始通道数
 * @param path input path, jpg/jpeg/png/bmp only / 输入路径，仅支持 jpg/jpeg/png/bmp
 * @param image receives the pixels / 接收像素
 * @param error optional message, written on failure / 失败时写入原因
 * @return true on success
 * @since 1.0
 * @exporter
 */
bool image_load(const std::string& path, Image& image,
                std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

/**
 * @brief [en] Save an uncompressed png from an 8-bit image
 * @brief [zh] 把 8 位图像保存为 png
 * @param path output path, png only / 输出路径，仅支持 png
 * @param image pixels to write / 待写出的像素
 * @param error optional message, written on failure / 失败时写入原因
 * @return true on success
 * @since 1.0
 * @exporter
 */
bool image_save_png(const std::string& path, const Image& image,
                    std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

/**
 * @brief [en] Save a disparity map; png is 16-bit x16 fixed point, tif is 32-bit float
 * @brief [zh] 保存视差图；png 为 16 位 x16 定点，tif 为 32 位浮点
 * @param path output path, png/tif/tiff / 输出路径，png/tif/tiff
 * @param disparity map in pixels, NaN marks invalid / 视差图，NaN 表示无效
 * @param error optional message, written on failure / 失败时写入原因
 * @return true on success
 * @since 1.0
 * @exporter
 */
bool disparity_save(const std::string& path, const DisparityImage& disparity,
                    std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

/**
 * @brief [en] Save a depth map; png is 16-bit millimetres, tif is 32-bit float metres
 * @brief [zh] 保存深度图；png 为 16 位毫米，tif 为 32 位浮点米
 * @param path output path, png/tif/tiff / 输出路径，png/tif/tiff
 * @param depth map in metres, NaN marks invalid / 深度图，NaN 表示无效
 * @param error optional message, written on failure / 失败时写入原因
 * @return true on success
 * @since 1.0
 * @exporter
 */
bool depth_save(const std::string& path, const DepthImage& depth,
                std::string* error); // NOLINT(bugprone-easily-swappable-parameters)

} // namespace sv
