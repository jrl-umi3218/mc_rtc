## Building from source without a script

Building from sources on other platforms is not well documented at the moment. If you wish to embark on this adventure, the following packages are required to build mc\_rtc:
- [CMake](https://cmake.org/) >= 3.1
- [Boost](https://www.boost.org/) >= 1.49
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) >= 3.2
- [tinyxml2](https://github.com/leethomason/tinyxml2)
- [GEOS](https://trac.osgeo.org/geos) (With C++ bindings)
- [LTDL](https://www.gnu.org/software/libtool/manual/html_node/Libltdl-interface.html) (Not required for Windows users)
- [nanomsg](https://github.com/nanomsg/nanomsg)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [spdlog](https://github.com/gabime/spdlog/) >= 1.6.1
- [hpp-spline](https://github.com/humanoid-path-planner/hpp-spline)
- [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg)
- [RBDyn](https://github.com/jrl-umi3218/RBDyn)
- [eigen-qld](https://github.com/jrl-umi3218/eigen-qld)
- [eigen-quadprog](https://github.com/jrl-umi3218/eigen-quadprog)
- [eigen-lssol](https://gite.lirmm.fr/multi-contact/eigen-lssol) (Optional)
- [sch-core](https://github.com/jrl-umi3218/sch-core)
- [Tasks](https://github.com/jrl-umi3218/Tasks)
- [mc_rtc_data](https://github.com/jrl-umi3218/mc_rtc_data)
- [state-observation](https://github.com/jrl-umi3218/state-observation)

mc\_rtc also has a ROS plugin that enables automated robot's status publication as ROS topics and facilitate the integration with ROS tools (e.g. RViZ), to build this you will need:

 * [mc_rtc_msgs](https://github.com/jrl-umi3218/mc_rtc_msgs)

If you wish to get Python bindings you will also need the following:
 * [Cython](http://cython.org/) >= 0.2
 * [python-pip]()
 * [python-numpy]()
 * [python-nose]()
 * [python-coverage]()
 * [Eigen3ToPython](https://github.com/jrl-umi3218/Eigen3ToPython)
 * [sch-core-python](https://github.com/jrl-umi3218/sch-core-python)

Additional Python libraries are required to run mc\_rtc tools:
 * [python-git]() (pip name: `GitPython`)
 * [python-pyqt5]()

The following packages are not required but bring additionnal features:
- [ROS](http://www.ros.org/)
- [mc\_rtc\_ros](https://github.com/jrl-umi3218/mc_rtc_ros)

If `roscpp` is available during build then `tf2_ros` and `sensor_msgs` are also required. This will add some integration between the `ROS` framework and `mc_rtc` (e.g. robot's state publication) and allow controllers to use ROS functionnalities (provides a `ros::NodeHandle` instance that can be used anywhere in `mc_rtc`).

## Getting started

Once mc_rtc has been installed, you can jump to the next [section]({{site.baseurl}}/tutorials/introduction/configuration.html).
