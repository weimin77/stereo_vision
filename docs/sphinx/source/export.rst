_`Full demonstration of specific lib`
=====================================

Brief overview: the library :code:`stereo_vision` turns a calibrated stereo pair into a disparity map, a
depth map and a point cloud. The chain is: epipolar rectification, SGBM matching, disparity to depth,
depth to point cloud.

_`Theories or introduction`
---------------------------

A rectified stereo pair observes one scene point from two positions. Once the epipolar lines are
horizontal, the horizontal coordinate difference of the match is the disparity :math:`d`, and depth
follows from the rectified geometry:

.. math::
   :label: depth from disparity

   Z = \frac{f_x \, |T_x|}{d - d_{offs}}, \qquad d_{offs} = c_{x1} - c_{x2}

:math:`f_x` is the rectified focal length, :math:`T_x` the baseline component along the camera x axis,
and :math:`c_{x1}, c_{x2}` the horizontal principal points of the two rectified views. The data term is
the matching cost of the SGBM energy, the smoothness term is the penalty paid when neighbouring pixels
take different labels:

.. math::

   E(D) = \sum_p C(p, D_p) + \sum_{q \in \mathcal{N}_p} P_1 \, \delta(\lvert D_p - D_q \rvert = 1)
        + \sum_{q \in \mathcal{N}_p} P_2 \, \delta(\lvert D_p - D_q \rvert > 1)

The energy is minimised along several scanline directions and the per-pixel minimum is kept, which is
what lets textureless regions take a plausible label instead of following noise. :math:`P_1` and
:math:`P_2` are derived from the block size: :math:`P_1 = p_1^{factor} b^2`,
:math:`P_2 = p_2^{factor} b^2`.

Three details make the difference between a usable result and a plausible-looking one:

- :math:`d_{offs}` is generally not zero. Dropping it shifts the whole cloud sideways by
  :math:`d_{offs} Z / f_x`, while the depth values themselves stay correct.
- Cropping to the common valid region translates the origin, so ``Q``'s fourth column and the principal
  points of ``P1``/``P2`` must move with it.
- Remapping a white image through the same maps marks the pixels that really carry data. Without that
  mask, the matcher reads the black border as scene content and invents disparity there.

_`Usage demonstration`
----------------------

The pipeline is a facade: images enter through ``set_input_image``, parameters through ``Config``,
and every artefact leaves through an explicit call. ``run()`` computes and writes nothing, so a caller
can inspect the result before deciding which files to produce.

.. code-block:: cpp
   :caption: stereo pipeline
   :name: stereo pipeline

   #include "sv_config.hpp"
   #include "sv_pipeline.hpp"

   int main() {
       sv::Config config;
       std::string error;
       sv::config_load("sgbm_config.json", config, &error);   // parameters only, no image paths

       sv::Pipeline pipeline{config};
       pipeline.set_calibration_files(config.stereo_yml, config.left_mono_yml, config.right_mono_yml);
       pipeline.set_input_image("left.png", "right.png");     // jpg, jpeg, png or bmp
       if (!pipeline.run()) {
           return 1;                                          // pipeline.last_error() holds the reason
       }

       const sv::RunReport& report = pipeline.report();
       // report.valid_ratio, report.disparity_p50, report.depth_p50, per-stage timings...

       pipeline.save_disparity_image("out/disparity.png");    // 16-bit, d * 16
       pipeline.save_depth_image("out/depth.png");            // 16-bit millimetres
       pipeline.save_disparity_image("out/disparity_color.png", true);   // 8-bit thermal ramp
       pipeline.save_depth_image("out/depth_color.png", true);           // 8-bit, near is warm
       pipeline.save_pointcloud("out/cloud.ply");             // ascii or binary per config
       return 0;
   }

_`Applying results`
-------------------

Five artefacts come out of a run, and each has its own consumer:

.. list-table::
   :header-rows: 1
   :widths: 26 34 40

   * - Artefact
     - Form
     - Typical use
   * - Disparity map
     - ``float`` px, NaN invalid; png is 1/16 px fixed point
     - Quality inspection, comparison against a matcher baseline
   * - Depth map
     - ``float`` m, NaN invalid; png is millimetres
     - Metric measurement, depth gating, near/far clipping
   * - Colourised disparity / depth
     - 8-bit BGR png, jpg or bmp; ``colorize = true``
     - Opening the result in a viewer; the same ramp for both maps, so near is warm in each
   * - Point cloud
     - PCD (``x y z rgb``) or PLY (``x y z`` + RGB)
     - Registration, plane/box fitting, bin picking
   * - Validity mask
     - ``uint8``, 255 usable
     - Excluding border and occluded pixels from downstream statistics

A 16-bit fixed-point or millimetre sample is the right thing to measure and the wrong thing to look at:
most viewers render it as a near-black frame. ``colorize = true`` writes an 8-bit ``COLORMAP_TURBO``
image instead, stretched between the 5th and 95th percentile of the valid samples so a handful of
outliers cannot flatten the whole map, with invalid pixels left black. It is a view, not a
measurement: the range depends on the data, so read numbers from the plain artefact. The colour
form needs an 8-bit container, so ``colorize`` with a ``.tif`` target is rejected rather than
silently writing something else.

The point cloud is expressed in the rectified left camera frame: x right, y down, z forward, in metres.
Because ``pointcloud.keep_invalid`` defaults to false, the cloud holds only pixels with a valid depth;
the full pixel grid can be rebuilt from the disparity map plus ``fx``, ``cx``, ``cy``, ``doffs`` and
``baseline``, all of which the run report records.

_`Algorithm benchmark from test frame`
--------------------------------------

Measured on a 640x480 synthetic pair with a known 20 px horizontal shift, ``fx = 500``, ``baseline =
0.1 m``, window ``16 + 16``; single frame, Release build:

.. table:: single-frame cost and agreement
   :name: test results
   :align: center

   ============== =========== ========================================
   stage          time        result
   ============== =========== ========================================
   rectify        4.9 ms      ``639x479`` after common-ROI cropping
   match          80.3 ms     median disparity ``20.00 px``
   depth          0.4 ms      median depth ``2.500 m``
   coverage       —           ``96.8 %`` of pixels valid
   point cloud    —           ``296355`` points
   ============== =========== ========================================

The median disparity equals the injected shift and the median depth equals
:math:`f_x \lvert T_x \rvert / d`, which is the property the unit tests assert. Suite level:
54 GTest cases, 81.3 % line coverage.
