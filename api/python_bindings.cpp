#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>
#include <string>

#include "sv_calibration.hpp"
#include "sv_config.hpp"
#include "sv_image_io.hpp"
#include "sv_pipeline.hpp"
#include "sv_types.hpp"


namespace py = pybind11;


namespace {

// The C++ side returns cached results by reference; pybind11 copies them into Python objects anyway,
// so the copy is made explicit instead of relying on the return-value policy.


sv::DisparityImage copy_disparity(const sv::Pipeline& pipeline) { return pipeline.get_disparity_image(); }

sv::DepthImage copy_depth(const sv::Pipeline& pipeline) { return pipeline.get_depth_image(); }

sv::PointCloud copy_cloud(const sv::Pipeline& pipeline) { return pipeline.get_pointcloud(); }


sv::Config load_config(const std::string& path) {
    sv::Config config;
    std::string error;
    if (!sv::config_load(path, config, &error)) {
        throw std::runtime_error(error);
    }
    return config;
}


/**
 * @brief A pipeline whose failures raise instead of returning False.
 *   The C++ API reports failures through bool + last_error() so that a caller can decide what to
 *   do; Python callers overwhelmingly expect an exception, so the binding translates.
 */


class PipelineHandle {
  public:
    explicit PipelineHandle(const sv::Config& config) : pipeline_(config) {}

    void set_calibration_files(const std::string& stereo_yml, const std::string& left_mono_yml,
                               const std::string& right_mono_yml) {
        if (!pipeline_.set_calibration_files(stereo_yml, left_mono_yml, right_mono_yml)) {
            throw std::runtime_error(pipeline_.last_error());
        }
    }

    void set_input_image(const std::string& left_path, const std::string& right_path) {
        if (!pipeline_.set_input_image(left_path, right_path)) {
            throw std::runtime_error(pipeline_.last_error());
        }
    }

    void run() {
        if (!pipeline_.run()) {
            throw std::runtime_error(pipeline_.last_error());
        }
    }

    void save_disparity_image(const std::string& path, bool colorize) const {
        if (!pipeline_.save_disparity_image(path, colorize)) {
            throw std::runtime_error(pipeline_.last_error());
        }
    }

    void save_depth_image(const std::string& path, bool colorize) const {
        if (!pipeline_.save_depth_image(path, colorize)) {
            throw std::runtime_error(pipeline_.last_error());
        }
    }

    void save_pointcloud(const std::string& path) const {
        if (!pipeline_.save_pointcloud(path)) {
            throw std::runtime_error(pipeline_.last_error());
        }
    }

    sv::DisparityImage disparity() const { return copy_disparity(pipeline_); }
    sv::DepthImage depth() const { return copy_depth(pipeline_); }
    sv::PointCloud pointcloud() const { return copy_cloud(pipeline_); }
    const sv::RunReport& report() const { return pipeline_.report(); }

  private:
    sv::Pipeline pipeline_;
};

} // namespace


PYBIND11_MODULE(stereo_vision_python, m) {
    m.doc() = "Python bindings for stereo_vision: rectify, match, depth and point cloud";

    py::class_<sv::Image>(m, "Image", "8-bit image, 1 (gray) or 3 (BGR) channels")
        .def(py::init<>())
        .def_readwrite("width", &sv::Image::width)
        .def_readwrite("height", &sv::Image::height)
        .def_readwrite("channels", &sv::Image::channels)
        .def_readwrite("data", &sv::Image::data)
        .def("valid", &sv::image_valid, "Whether size, channel count and buffer agree");

    py::class_<sv::DisparityImage>(m, "DisparityImage", "Sub-pixel disparity in pixels; NaN is invalid")
        .def(py::init<>())
        .def_readwrite("width", &sv::DisparityImage::width)
        .def_readwrite("height", &sv::DisparityImage::height)
        .def_readwrite("data", &sv::DisparityImage::data)
        .def("valid", &sv::disparity_valid);

    py::class_<sv::DepthImage>(m, "DepthImage", "Depth in metres; NaN is invalid")
        .def(py::init<>())
        .def_readwrite("width", &sv::DepthImage::width)
        .def_readwrite("height", &sv::DepthImage::height)
        .def_readwrite("data", &sv::DepthImage::data)
        .def("valid", &sv::depth_valid);

    py::class_<sv::Point>(m, "Point", "One back-projected point")
        .def(py::init<>())
        .def_readwrite("x", &sv::Point::x)
        .def_readwrite("y", &sv::Point::y)
        .def_readwrite("z", &sv::Point::z);

    py::class_<sv::PointCloud>(m, "PointCloud", "Points in the rectified left camera frame")
        .def(py::init<>())
        .def_readwrite("points", &sv::PointCloud::points);

    py::class_<sv::RectifyGeometry>(m, "RectifyGeometry", "Geometry of the rectified views")
        .def(py::init<>())
        .def_readwrite("width", &sv::RectifyGeometry::width)
        .def_readwrite("height", &sv::RectifyGeometry::height)
        .def_readwrite("fx", &sv::RectifyGeometry::fx)
        .def_readwrite("fy", &sv::RectifyGeometry::fy)
        .def_readwrite("cx", &sv::RectifyGeometry::cx)
        .def_readwrite("cy", &sv::RectifyGeometry::cy)
        .def_readwrite("doffs", &sv::RectifyGeometry::doffs)
        .def_readwrite("baseline", &sv::RectifyGeometry::baseline)
        .def_readwrite("focal_baseline", &sv::RectifyGeometry::focal_baseline);

    py::class_<sv::RunReport>(m, "RunReport", "Timings and statistics of one run")
        .def(py::init<>())
        .def_readwrite("rectify_ms", &sv::RunReport::rectify_ms)
        .def_readwrite("match_ms", &sv::RunReport::match_ms)
        .def_readwrite("depth_ms", &sv::RunReport::depth_ms)
        .def_readwrite("rectified_width", &sv::RunReport::rectified_width)
        .def_readwrite("rectified_height", &sv::RunReport::rectified_height)
        .def_readwrite("min_disparity", &sv::RunReport::min_disparity)
        .def_readwrite("num_disparities", &sv::RunReport::num_disparities)
        .def_readwrite("valid_ratio", &sv::RunReport::valid_ratio)
        .def_readwrite("disparity_p05", &sv::RunReport::disparity_p05)
        .def_readwrite("disparity_p50", &sv::RunReport::disparity_p50)
        .def_readwrite("disparity_p95", &sv::RunReport::disparity_p95)
        .def_readwrite("depth_p05", &sv::RunReport::depth_p05)
        .def_readwrite("depth_p50", &sv::RunReport::depth_p50)
        .def_readwrite("depth_p95", &sv::RunReport::depth_p95)
        .def_readwrite("output_directory_resolved", &sv::RunReport::output_directory_resolved);

    py::enum_<sv::SgbmMode>(m, "SgbmMode", "Aggregation mode of cv::StereoSGBM")
        .value("SGBM", sv::SgbmMode::Sgbm)
        .value("HH", sv::SgbmMode::Hh)
        .value("HH4", sv::SgbmMode::Hh4)
        .value("THREE_WAY", sv::SgbmMode::ThreeWay);

    py::class_<sv::SgbmParams>(m, "SgbmParams", "SGBM parameters")
        .def(py::init<>())
        .def_readwrite("min_disparity", &sv::SgbmParams::min_disparity)
        .def_readwrite("num_disparities", &sv::SgbmParams::num_disparities)
        .def_readwrite("block_size", &sv::SgbmParams::block_size)
        .def_readwrite("p1_factor", &sv::SgbmParams::p1_factor)
        .def_readwrite("p2_factor", &sv::SgbmParams::p2_factor)
        .def_readwrite("pre_filter_cap", &sv::SgbmParams::pre_filter_cap)
        .def_readwrite("uniqueness_ratio", &sv::SgbmParams::uniqueness_ratio)
        .def_readwrite("disp12_max_diff", &sv::SgbmParams::disp12_max_diff)
        .def_readwrite("speckle_window_size", &sv::SgbmParams::speckle_window_size)
        .def_readwrite("speckle_range", &sv::SgbmParams::speckle_range)
        .def_readwrite("mode", &sv::SgbmParams::mode)
        .def_readwrite("padding", &sv::SgbmParams::padding)
        .def_readwrite("clahe", &sv::SgbmParams::clahe)
        .def_readwrite("clahe_clip_limit", &sv::SgbmParams::clahe_clip_limit)
        .def_readwrite("clahe_tile_grid", &sv::SgbmParams::clahe_tile_grid);

    py::class_<sv::RectifyOptions>(m, "RectifyOptions", "Rectification options")
        .def(py::init<>())
        .def_readwrite("enabled", &sv::RectifyOptions::enabled)
        .def_readwrite("alpha", &sv::RectifyOptions::alpha)
        .def_readwrite("new_width", &sv::RectifyOptions::new_width)
        .def_readwrite("new_height", &sv::RectifyOptions::new_height)
        .def_readwrite("crop_to_common_roi", &sv::RectifyOptions::crop_to_common_roi)
        .def_readwrite("fixed_point_maps", &sv::RectifyOptions::fixed_point_maps);

    py::class_<sv::Config>(m, "Config", "Everything sgbm_config.json can say about a run")
        .def(py::init<>())
        .def_readwrite("profile", &sv::Config::profile)
        .def_readwrite("matcher", &sv::Config::matcher)
        .def_readwrite("rectify", &sv::Config::rectify)
        .def_readwrite("stereo_yml", &sv::Config::stereo_yml)
        .def_readwrite("left_mono_yml", &sv::Config::left_mono_yml)
        .def_readwrite("right_mono_yml", &sv::Config::right_mono_yml)
        .def_readwrite("working_near_m", &sv::Config::working_near_m)
        .def_readwrite("working_far_m", &sv::Config::working_far_m)
        .def_readwrite("depth_min_m", &sv::Config::depth_min_m)
        .def_readwrite("depth_max_m", &sv::Config::depth_max_m)
        .def_readwrite("cloud_binary", &sv::Config::cloud_binary)
        .def_readwrite("cloud_keep_invalid", &sv::Config::cloud_keep_invalid)
        .def_readwrite("output_directory", &sv::Config::output_directory)
        .def_readwrite("output_prefix", &sv::Config::output_prefix)
        .def_readwrite("warnings", &sv::Config::warnings);

    m.def("config_default", &sv::config_default, "A configuration holding only the built-in defaults");
    m.def("config_load", &load_config, py::arg("path"), "Parse a JSON configuration; raises on failure");

    m.def(
        "image_load",
        [](const std::string& path) {
            sv::Image image;
            std::string error;
            if (!sv::image_load(path, image, &error)) {
                throw std::runtime_error(error);
            }
            return image;
        },
        py::arg("path"), "Load jpg, jpeg, png or bmp; raises on failure");

    m.def(
        "image_save_png",
        [](const std::string& path, const sv::Image& image) {
            std::string error;
            if (!sv::image_save_png(path, image, &error)) {
                throw std::runtime_error(error);
            }
        },
        py::arg("path"), py::arg("image"));

    m.def(
        "disparity_save",
        [](const std::string& path, const sv::DisparityImage& disparity, bool colorize) {
            std::string error;
            if (!sv::disparity_save(path, disparity, &error, colorize)) {
                throw std::runtime_error(error);
            }
        },
        py::arg("path"), py::arg("disparity"), py::arg("colorize") = false);

    m.def(
        "depth_save",
        [](const std::string& path, const sv::DepthImage& depth, bool colorize) {
            std::string error;
            if (!sv::depth_save(path, depth, &error, colorize)) {
                throw std::runtime_error(error);
            }
        },
        py::arg("path"), py::arg("depth"), py::arg("colorize") = false);

    m.def(
        "pointcloud_save",
        [](const std::string& path, const sv::PointCloud& cloud, bool binary) {
            std::string error;
            if (!sv::pointcloud_save(path, cloud, binary, &error)) {
                throw std::runtime_error(error);
            }
        },
        py::arg("path"), py::arg("cloud"), py::arg("binary") = true);

    py::class_<PipelineHandle>(m, "Pipeline", "Rectify, match, convert to depth, expose results")
        .def(py::init<const sv::Config&>(), py::arg("config"))
        .def("set_calibration_files", &PipelineHandle::set_calibration_files, py::arg("stereo_yml"),
             py::arg("left_mono_yml") = std::string(), py::arg("right_mono_yml") = std::string())
        .def("set_input_image", &PipelineHandle::set_input_image, py::arg("left_path"), py::arg("right_path"))
        .def("run", &PipelineHandle::run, "Rectify, match and convert to depth; writes nothing")
        .def("disparity", &PipelineHandle::disparity, "Cached disparity map (copy)")
        .def("depth", &PipelineHandle::depth, "Cached depth map (copy)")
        .def("pointcloud", &PipelineHandle::pointcloud, "Point cloud built from the depth map (copy)")
        .def("save_disparity_image", &PipelineHandle::save_disparity_image, py::arg("path"),
             py::arg("colorize") = false)
        .def("save_depth_image", &PipelineHandle::save_depth_image, py::arg("path"), py::arg("colorize") = false)
        .def("save_pointcloud", &PipelineHandle::save_pointcloud, py::arg("path"))
        .def("report", &PipelineHandle::report, py::return_value_policy::copy);

    m.attr("__version__") = "0.1.0";
}
