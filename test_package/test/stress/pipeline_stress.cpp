#include <gtest/gtest.h>

#include <cmath>

#include "sv_pipeline.hpp"
#include "synthetic.hpp"

namespace {

constexpr int kIterations = 3;
constexpr int kShift = 16;
constexpr int kNumDisparities = 64;

}  // namespace

TEST(PipelineStress, RepeatedRunsStayStable) {
    sv::Config config = sv::config_default();
    config.matcher.num_disparities = kNumDisparities;
    config.matcher.mode = sv::SgbmMode::Sgbm;
    config.rectify.crop_to_common_roi = false;

    sv::Pipeline pipeline{config};
    const sv::Image left = synthetic::blocky_texture(320, 240, 31U);
    const sv::Image right = synthetic::shift_horizontal(left, kShift);
    ASSERT_TRUE(pipeline.set_calibration(synthetic::pinhole_rig()));
    ASSERT_TRUE(pipeline.set_input_image(left, right));

    for (int i = 0; i < kIterations; ++i) {
        ASSERT_TRUE(pipeline.run()) << pipeline.last_error();
        const double median = synthetic::median_of_finite(pipeline.get_disparity_image().data);
        EXPECT_TRUE(std::isfinite(median));
    }
}
