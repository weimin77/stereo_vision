_`stereo_vision Documentation`
==============================

_`Introduction`
---------------

stereo_vision turns a calibrated stereo pair into a disparity map, a depth map and a point cloud:
epipolar rectification, SGBM matching, disparity to depth, depth to point cloud. It is built with the
same C/C++ build system as the fcpp template (Conan 2, CMake, Doxygen + Sphinx).

_`Main Features`
----------------

- Rectification with common-ROI cropping, rebased Q/P and a validity mask
- cv::StereoSGBM wrapper with padding, 1/16 px unpacking and a left-right check
- Depth from the rectified geometry, then a point cloud in the left camera frame
- PCD and PLY output without a PCL dependency
- Conan-based dependency management, cross-platform build support

_`Quick Start`
--------------

.. code-block:: bash

   # Install dependencies
   conan install .
   
   # Build the project
   conan build .

   # Create the package
   conan create .

_`Project Structure`
--------------------

::

   stereo_vision/
   ├── include/         # Public headers (sv_*.hpp)
   ├── src/             # Implementations (sv_*.cpp)
   ├── test_package/    # GTest unit/stress tests and the smoke program
   ├── tools/stereo_cli # Command-line consumer of the packaged library
   ├── CMakeLists.txt   # CMake build configuration
   ├── conanfile.py     # Conan package configuration
   ├── metadata.json    # Project metadata
   └── LICENSE          # License file

_`Detailed Settings`
--------------------

For detailed settings in project-level meta, conan recipe, or cmake file, see:

.. toctree::
   CMakeList <components/cmake>
   Conanfile <components/conan>
   Metadata <components/meta>
   User concern contents <export>
   :numbered:
   :maxdepth: 2

_`License`
----------

This project is licensed under the Apache-2.0 License. See the LICENSE file for details.
