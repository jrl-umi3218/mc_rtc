#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(MC_RTC_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include")

set(VERSION_SCRIPT_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/version-script.lds")
set(MC_RTC_ALL_TARGETS)

if(UNIX AND NOT EMSCRIPTEN)
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-msse2" SSE2_SUPPORTED)
endif()

macro(install_mc_rtc_lib_common MC_LIB KWD)
  target_compile_features(${MC_LIB} ${KWD} cxx_std_17)
  target_include_directories(
    ${MC_LIB} INTERFACE $<INSTALL_INTERFACE:include>
                        $<BUILD_INTERFACE:${MC_RTC_INCLUDE_DIR}>
  )
  if(MSVC)
    target_compile_options(${MC_LIB} ${KWD} /source-charset:utf-8)
  endif()
  if(NOT EMSCRIPTEN)
    if(WIN32)
      target_compile_definitions(
        ${MC_LIB} ${KWD} "-DNOMINMAX" "-D_ENABLE_EXTENDED_ALIGNED_STORAGE"
      )
    else()
      if(SSE2_SUPPORTED)
        target_compile_options(${MC_LIB} ${KWD} "-mfpmath=sse" "-msse2")
      endif()
    endif()
  else()
    target_compile_options(${MC_LIB} ${KWD} -matomics -s USE_PTHREADS=1)
  endif()
  target_compile_features(${MC_LIB} ${KWD} cxx_std_17)
  if(TREAT_WARNINGS_AS_ERRORS)
    if(MSVC)
      target_compile_options(${MC_LIB} ${KWD} $<BUILD_INTERFACE:/WX>)
    else()
      target_compile_options(${MC_LIB} ${KWD} $<BUILD_INTERFACE:-Werror>)
    endif()
  endif()
  if(MC_RTC_BUILD_STATIC)
    target_compile_definitions(${MC_LIB} ${KWD} MC_RTC_BUILD_STATIC)
  endif()
  add_library(mc_rtc::${MC_LIB} ALIAS ${MC_LIB})
  list(APPEND MC_RTC_ALL_TARGETS ${MC_LIB})
  set(MC_RTC_ALL_TARGETS
      "${MC_RTC_ALL_TARGETS}"
      PARENT_SCOPE
  )
endmacro()

macro(install_mc_rtc_header_lib MC_LIB)
  install_mc_rtc_lib_common(${MC_LIB} INTERFACE)
  jrl_add_export_component(NAME ${MC_LIB} TARGETS ${MC_LIB})
  # install(TARGETS ${MC_LIB} EXPORT "${TARGETS_EXPORT_NAME}")
endmacro()

macro(install_mc_rtc_lib MC_LIB)
  set_target_properties(${MC_LIB} PROPERTIES FOLDER mc_libraries/${MC_LIB})
  install_mc_rtc_lib_common(${MC_LIB} PUBLIC)
  if(MSVC)
    target_compile_options(${MC_LIB} PRIVATE "/bigobj")
  endif()
  if(GENERATE_COVERAGE)
    target_compile_options(${MC_LIB} PRIVATE "--coverage")
    set_target_properties(${MC_LIB} PROPERTIES LINK_FLAGS "--coverage")
  endif()
  set_target_properties(
    ${MC_LIB} PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION}
  )
  if(NOT MC_RTC_BUILD_STATIC)
    # add_version_script(${MC_LIB} "${VERSION_SCRIPT_LOCATION}") XXX: Check if this is
    # strictly equivalent to the old version script
    jrl_target_generate_config_header(
      ${MC_LIB} PUBLIC VERSION "${PROJECT_VERSION}" FILENAME "${MC_LIB}/api.h"
    )
    jrl_target_generate_deprecated_header(
      ${MC_LIB} PUBLIC FILENAME "${MC_LIB}/deprecated.h"
    )
  endif()
  jrl_add_export_component(NAME ${MC_LIB} TARGETS ${MC_LIB})
endmacro()

configure_file(mc_rtc/loader.in.cpp "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc/loader.cpp")

set(VERSION_HEADER "${PROJECT_BINARY_DIR}/include/mc_rtc/version.h")
add_custom_command(
  OUTPUT ${VERSION_HEADER}
  COMMAND
    ${CMAKE_COMMAND} -DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
    -DPROJECT_BINARY_DIR="${PROJECT_BINARY_DIR}" -DPROJECT_VERSION="${PROJECT_VERSION}"
    -DPROJECT_VERSION_MAJOR="${PROJECT_VERSION_MAJOR}"
    -DMC_RTC_DEVELOPER_MODE="${MC_RTC_DEVELOPER_MODE}" -P
    "${PROJECT_SOURCE_DIR}/CMakeModules/MakeVersionHeader.cmake"
)
add_custom_target(generate_version_header DEPENDS ${VERSION_HEADER})

set(DEBUG_SOURCE "${PROJECT_BINARY_DIR}/src/mc_rtc/$<LOWER_CASE:$<CONFIG>>/debug.cpp")
# This is not necessary but a work-around for
# https://gitlab.kitware.com/cmake/cmake/-/issues/21203
configure_file(mc_rtc/debug.in.cpp "${PROJECT_BINARY_DIR}/src/mc_rtc/debug.cpp")
file(
  GENERATE
  OUTPUT ${DEBUG_SOURCE}
  INPUT "${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc/debug.in.cpp"
)

set(mc_rtc_utils_SRC
    mc_rtc/Configuration.cpp
    mc_rtc/ConfigurationHelpers.cpp
    mc_rtc/DataStore.cpp
    mc_rtc/FlatLog.cpp
    mc_rtc/iterate_binary_log.cpp
    mc_rtc/Logger.cpp
    mc_rtc/MessagePackBuilder.cpp
    mc_rtc/deprecated.cpp
    mc_rtc/logging.cpp
    mc_rtc/path.cpp
    mc_rtc/version.cpp
    ${DEBUG_SOURCE}
)
add_library(mc_rtc_utils SHARED ${mc_rtc_utils_SRC})

jrl_target_headers(
  mc_rtc_utils
  PUBLIC
  HEADERS
  include/mc_rtc/Configuration.h
  include/mc_rtc/ConfigurationHelpers.h
  include/mc_rtc/MessagePackBuilder.h
  include/mc_rtc/logging.h
  include/mc_rtc/fmt_formatters.h
  include/mc_rtc/log/FlatLog.h
  include/mc_rtc/log/iterate_binary_log.h
  include/mc_rtc/log/Logger.h
  include/mc_rtc/io_utils.h
  include/mc_rtc/utils.h
  include/mc_rtc/constants.h
  include/mc_rtc/DataStore.h
  include/mc_rtc/type_name.h
  include/mc_rtc/debug.h
  include/mc_rtc/deprecated.h
  include/mc_rtc/path.h
  include/mc_rtc/pragma.h
  include/mc_rtc/shared.h
  include/mc_rtc/visual_utils.h
  include/mc_rtc/utils/heatmap.h
  include/mc_rtc/clock.h
  include/mc_rtc/threading/AsyncJob.h
  include/mc_rtc/SignalSlot.h
  BASE_DIRS
  include
)
jrl_target_headers(
  mc_rtc_utils
  PUBLIC
  HEADERS
  src/mc_rtc/internals/json.h
  src/mc_rtc/internals/msgpack.h
  src/mc_rtc/internals/yaml.h
  src/mc_rtc/internals/LogEntry.h
  BASE_DIRS
  src
)
jrl_target_headers(
  mc_rtc_utils
  PRIVATE
  HEADERS
  "${VERSION_HEADER}"
  "${PROJECT_BINARY_DIR}/include/mc_rtc/config.h"
  BASE_DIRS
  ${PROJECT_BINARY_DIR}/include
)
jrl_target_headers(
  mc_rtc_utils PUBLIC HEADERS ${PROJECT_BINARY_DIR}/include/mc_rtc/config.h BASE_DIRS
  ${PROJECT_BINARY_DIR}/include
)

add_dependencies(mc_rtc_utils generate_version_header)
target_include_directories(mc_rtc_utils PRIVATE "${PROJECT_BINARY_DIR}/include/mc_rtc")
target_include_directories(
  mc_rtc_utils
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>
)
target_compile_definitions(mc_rtc_utils PUBLIC ${BOOST_USE_STACKTRACE_DEFINE})
target_link_libraries(
  mc_rtc_utils
  PUBLIC SpaceVecAlg::SpaceVecAlg
         Boost::${BOOST_STACKTRACE}
         Boost::filesystem
         Boost::disable_autolinking
         ${YAML_CPP_TARGET}
         spdlog::spdlog
         ${CMAKE_DL_LIBS}
  PRIVATE $<BUILD_INTERFACE:mc_rtc_3rd_party::RapidJSON>
)
if(TARGET mc_rtc_3rd_party::ROS)
  target_link_libraries(mc_rtc_utils PRIVATE mc_rtc_3rd_party::ROS)
endif()
if(TARGET PkgConfig::mc_rtc_3rd_party_libnotify)
  target_link_libraries(mc_rtc_utils PUBLIC PkgConfig::mc_rtc_3rd_party_libnotify)
  target_compile_definitions(mc_rtc_utils PRIVATE MC_RTC_HAS_LIBNOTIFY)
elseif(HOST_IS_WSL)
  target_compile_definitions(mc_rtc_utils PRIVATE MC_RTC_HAS_WSL_NOTIFY_SEND)
  configure_file(
    mc_rtc/internals/logging.in.h
    "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc/internals/logging.h"
  )
  target_include_directories(mc_rtc_utils PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc")
else()
  set(WinToast_DIR "${PROJECT_SOURCE_DIR}/3rd-party/WinToast/src")
  target_sources(
    mc_rtc_utils PRIVATE ${WinToast_DIR}/wintoastlib.cpp ${WinToast_DIR}/wintoastlib.h
  )
  target_include_directories(mc_rtc_utils PRIVATE "${WinToast_DIR}")
  target_compile_definitions(mc_rtc_utils PRIVATE MC_RTC_HAS_WINTOAST)
endif()
if(NOT Boost_USE_STATIC_LIBS)
  target_link_libraries(mc_rtc_utils PUBLIC Boost::dynamic_linking)
endif()
if(NOT MC_RTC_BUILD_STATIC)
  target_link_libraries(mc_rtc_utils PRIVATE mpack)
else()
  target_sources(mc_rtc_utils PRIVATE ${PROJECT_SOURCE_DIR}/3rd-party/mpack/mpack.c)
  target_include_directories(
    mc_rtc_utils PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd-party/mpack>
  )
endif()
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" AND NOT EMSCRIPTEN)
  target_link_libraries(mc_rtc_utils PUBLIC atomic)
endif()
install_mc_rtc_lib(mc_rtc_utils)

set(mc_rtc_loader_SRC "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc/loader.cpp")

set(mc_rtc_loader_HDR ../include/mc_rtc/loader.h ../include/mc_rtc/loader_api.h)

add_library(mc_rtc_loader SHARED ${mc_rtc_loader_SRC} ${mc_rtc_loader_HDR})
target_link_libraries(
  mc_rtc_loader PUBLIC Boost::filesystem Boost::disable_autolinking mc_rtc_utils
)
if(NOT Boost_USE_STATIC_LIBS)
  target_link_libraries(mc_rtc_loader PUBLIC Boost::dynamic_linking)
endif()
if(NOT MC_RTC_BUILD_STATIC)
  target_link_libraries(mc_rtc_loader PUBLIC ${LTDL})
endif()
install_mc_rtc_lib(mc_rtc_loader)

set(mc_rbdyn_SRC
    mc_rbdyn/SCHAddon.cpp
    mc_rbdyn/contact_transform.cpp
    mc_rbdyn/Surface.cpp
    mc_rbdyn/PlanarSurface.cpp
    mc_rbdyn/CylindricalSurface.cpp
    mc_rbdyn/GripperSurface.cpp
    mc_rbdyn/surface_utils.cpp
    mc_rbdyn/surface_hull.cpp
    mc_rbdyn/Contact.cpp
    mc_rbdyn/Robot.cpp
    mc_rbdyn/Robots.cpp
    mc_rbdyn/PolygonInterpolator.cpp
    mc_rbdyn/polygon_utils.cpp
    mc_rbdyn/RobotLoader.cpp
    mc_rbdyn/RobotConverter.cpp
    mc_rbdyn/Collision.cpp
    mc_rbdyn/ForceSensor.cpp
    mc_rbdyn/RobotModule.cpp
    mc_rbdyn/RobotModule_visual.cpp
    mc_rbdyn/inertia.cpp
    mc_rbdyn/RobotModule_connect.cpp
    mc_rbdyn/configuration_io.cpp
    mc_rbdyn/ZMP.cpp
    mc_control/generic_gripper.cpp
    mc_rbdyn/Device.cpp
    mc_rbdyn/BodySensor.cpp
    mc_rbdyn/Frame.cpp
    mc_rbdyn/RobotFrame.cpp
)

add_library(mc_rbdyn SHARED)
target_sources(mc_rbdyn PRIVATE ${mc_rbdyn_SRC})
jrl_target_headers(
  mc_rbdyn
  PUBLIC
  HEADERS
  include/mc_rbdyn/fwd.h
  include/mc_rbdyn/hat.h
  include/mc_rbdyn/Contact.h
  include/mc_rbdyn/contact_transform.h
  include/mc_rbdyn/CylindricalSurface.h
  include/mc_rbdyn/GripperSurface.h
  include/mc_rbdyn/Mimic.h
  include/mc_rbdyn/PlanarSurface.h
  include/mc_rbdyn/polygon_utils.h
  include/mc_rbdyn/PolygonInterpolator.h
  include/mc_rbdyn/Robot.h
  include/mc_rbdyn/Robots.h
  include/mc_rbdyn/RobotLoader.h
  include/mc_rbdyn/RobotConverter.h
  include/mc_rbdyn/RobotModule.h
  include/mc_rbdyn/RobotModule_visual.h
  include/mc_rbdyn/inertia.h
  include/mc_rbdyn/RobotModuleMacros.h
  include/mc_rbdyn/SCHAddon.h
  include/mc_rbdyn/Surface.h
  include/mc_rbdyn/rpy_utils.h
  include/mc_rbdyn/surface_hull.h
  include/mc_rbdyn/surface_utils.h
  include/mc_rbdyn/Collision.h
  include/mc_rbdyn/CompoundJointConstraintDescription.h
  include/mc_rbdyn/ForceSensor.h
  include/mc_rbdyn/Flexibility.h
  include/mc_rbdyn/Springs.h
  include/mc_rbdyn/Base.h
  include/mc_rbdyn/BodySensor.h
  include/mc_rbdyn/configuration_io.h
  include/mc_rbdyn/lipm_stabilizer/StabilizerConfiguration.h
  include/mc_rbdyn/lipm_stabilizer/ZMPCCConfiguration.h
  include/mc_rbdyn/ZMP.h
  include/mc_control/generic_gripper.h
  include/mc_rbdyn/Device.h
  include/mc_rbdyn/Frame.h
  include/mc_rbdyn/RobotFrame.h
  include/mc_rbdyn/JointSensor.h
)

# tvm is added on purpose to the rbdyn target because it depends on rbdyn
set(mc_tvm_HDR_DIR ${PROJECT_SOURCE_DIR}/include/mc_tvm)
set(mc_tvm_HDR
    include/mc_tvm/CollisionFunction.h
    include/mc_tvm/CoM.h
    include/mc_tvm/CoMFunction.h
    include/mc_tvm/CoMInConvexFunction.h
    include/mc_tvm/CompoundJointFunction.h
    include/mc_tvm/ContactFunction.h
    include/mc_tvm/Convex.h
    include/mc_tvm/DynamicFunction.h
    include/mc_tvm/Frame.h
    include/mc_tvm/FrameVelocity.h
    include/mc_tvm/GazeFunction.h
    include/mc_tvm/Limits.h
    include/mc_tvm/JointsSelectorFunction.h
    include/mc_tvm/Momentum.h
    include/mc_tvm/MomentumFunction.h
    include/mc_tvm/OrientationFunction.h
    include/mc_tvm/PositionFunction.h
    include/mc_tvm/PositionBasedVisServoFunction.h
    include/mc_tvm/PostureFunction.h
    include/mc_tvm/Robot.h
    include/mc_tvm/RobotFrame.h
    include/mc_tvm/TransformFunction.h
    include/mc_tvm/VectorOrientationFunction.h
)
jrl_target_headers(mc_rbdyn PUBLIC HEADERS ${mc_tvm_HDR} BASE_DIRS include)
set(mc_tvm_SRC
    mc_tvm/CollisionFunction.cpp
    mc_tvm/CoM.cpp
    mc_tvm/CoMFunction.cpp
    mc_tvm/CoMInConvexFunction.cpp
    mc_tvm/CompoundJointFunction.cpp
    mc_tvm/ContactFunction.cpp
    mc_tvm/Convex.cpp
    mc_tvm/DynamicFunction.cpp
    mc_tvm/Frame.cpp
    mc_tvm/FrameVelocity.cpp
    mc_tvm/GazeFunction.cpp
    mc_tvm/JointsSelectorFunction.cpp
    mc_tvm/Momentum.cpp
    mc_tvm/MomentumFunction.cpp
    mc_tvm/OrientationFunction.cpp
    mc_tvm/PositionFunction.cpp
    mc_tvm/PositionBasedVisServoFunction.cpp
    mc_tvm/PostureFunction.cpp
    mc_tvm/Robot.cpp
    mc_tvm/RobotFrame.cpp
    mc_tvm/TransformFunction.cpp
    mc_tvm/VectorOrientationFunction.cpp
)
target_sources(mc_rbdyn PRIVATE ${mc_tvm_SRC})
target_link_libraries(
  mc_rbdyn
  PUBLIC RBDyn::Parsers
         Tasks::Tasks
         mc_rtc_loader
         mc_rtc_utils
         mc_filter
         GEOS::geos
         TVM::TVM
         mesh_sampling::mesh_sampling
)
target_compile_definitions(mc_rbdyn PUBLIC -DUSE_UNSTABLE_GEOS_CPP_API)
# qhull
if(MC_RTC_BUILD_STATIC)
  target_link_libraries(
    mc_rbdyn PRIVATE "-Wl,--no-whole-archive" Qhull::qhullstatic_r
                     "-Wl,--no-whole-archive" Qhull::qhullcpp
  )
else()
  target_link_libraries(mc_rbdyn PRIVATE Qhull::qhullstatic_r Qhull::qhullcpp)
endif()

if(TARGET mc_rtc_3rd_party::ROS)
  target_link_libraries(mc_rbdyn PRIVATE mc_rtc_3rd_party::ROS)
endif()

install_mc_rtc_lib(mc_rbdyn)

add_library(mc_rtc_gui SHARED)
set(mc_rtc_gui_SRC mc_rtc/gui/StateBuilder.cpp mc_rbdyn/gui/RobotConvex.cpp
                   mc_rbdyn/gui/RobotSurface.cpp
)

set(mc_rtc_gui_HDR
    include/mc_rtc/gui/StateBuilder.h
    include/mc_rtc/gui/StateBuilder.hpp
    include/mc_rtc/gui/ArrayInput.h
    include/mc_rtc/gui/ArrayLabel.h
    include/mc_rtc/gui/Arrow.h
    include/mc_rtc/gui/Box.h
    include/mc_rtc/gui/Button.h
    include/mc_rtc/gui/Checkbox.h
    include/mc_rtc/gui/ComboInput.h
    include/mc_rtc/gui/Cylinder.h
    include/mc_rtc/gui/DataComboInput.h
    include/mc_rtc/gui/elements.h
    include/mc_rtc/gui/Ellipsoid.h
    include/mc_rtc/gui/Force.h
    include/mc_rtc/gui/Form.h
    include/mc_rtc/gui/IntegerInput.h
    include/mc_rtc/gui/Input.h
    include/mc_rtc/gui/Label.h
    include/mc_rtc/gui/NumberInput.h
    include/mc_rtc/gui/NumberSlider.h
    include/mc_rtc/gui/Point3D.h
    include/mc_rtc/gui/Polygon.h
    include/mc_rtc/gui/Rotation.h
    include/mc_rtc/gui/Schema.h
    include/mc_rtc/gui/Sphere.h
    include/mc_rtc/gui/StringInput.h
    include/mc_rtc/gui/Trajectory.h
    include/mc_rtc/gui/Transform.h
    include/mc_rtc/gui/types.h
    include/mc_rtc/gui/Visual.h
    include/mc_rtc/gui/XYTheta.h
    include/mc_rtc/gui/details/traits.h
    include/mc_rtc/gui/plot.h
    include/mc_rtc/gui/plot/Abscissa.h
    include/mc_rtc/gui/plot/AbscissaOrdinate.h
    include/mc_rtc/gui/plot/types.h
    include/mc_rtc/gui/plot/Ordinate.h
    include/mc_rtc/gui/plot/Polygon.h
    include/mc_rtc/gui/plot/Polygons.h
    include/mc_rbdyn/gui/RobotConvex.h
    include/mc_rbdyn/gui/RobotSurface.h
)
target_sources(mc_rtc_gui PRIVATE ${mc_rtc_gui_SRC})
jrl_target_headers(mc_rtc_gui PUBLIC HEADERS ${mc_rtc_gui_HDR} BASE_DIRS include)
target_link_libraries(mc_rtc_gui PUBLIC mc_rbdyn)
install_mc_rtc_lib(mc_rtc_gui)

add_subdirectory(mc_robots)

set(mc_solver_SRC
    mc_solver/BoundedSpeedConstr.cpp
    mc_solver/CollisionsConstraint.cpp
    mc_solver/CoMIncPlaneConstr.cpp
    mc_solver/ConstraintSet.cpp
    mc_solver/CompoundJointConstraint.cpp
    mc_solver/ConstraintSetLoader.cpp
    mc_solver/ContactConstraint.cpp
    mc_solver/ContactWrenchMatrixToLambdaMatrix.cpp
    mc_solver/DynamicsConstraint.cpp
    mc_solver/KinematicsConstraint.cpp
    mc_solver/QPSolver.cpp
    mc_solver/TasksQPSolver.cpp
    mc_solver/TVMQPSolver.cpp
    mc_solver/Update.cpp
    mc_solver/UpdateNrVars.cpp
    mc_tasks/MetaTask.cpp
)

set(mc_solver_HDR
    include/mc_solver/utils/Constraint.h
    include/mc_solver/utils/ContactWrenchMatrixToLambdaMatrix.h
    include/mc_solver/utils/Update.h
    include/mc_solver/utils/UpdateNrVars.h
    include/mc_solver/BoundedSpeedConstr.h
    include/mc_solver/CollisionsConstraint.h
    include/mc_solver/CoMIncPlaneConstr.h
    include/mc_solver/CompoundJointConstraint.h
    include/mc_solver/ConstraintSet.h
    include/mc_solver/ContactConstraint.h
    include/mc_solver/DynamicsConstraint.h
    include/mc_solver/EqualityConstraint.h
    include/mc_solver/GenInequalityConstraint.h
    include/mc_solver/InequalityConstraint.h
    include/mc_solver/KinematicsConstraint.h
    include/mc_solver/TVMKinematicsConstraint.h
    include/mc_solver/GenericLoader.h
    include/mc_solver/GenericLoader.hpp
    include/mc_solver/GenInequalityConstraint.h
    include/mc_solver/InequalityConstraint.h
    include/mc_solver/QPSolver.h
    include/mc_solver/TasksQPSolver.h
    include/mc_solver/TVMQPSolver.h
    include/mc_tasks/MetaTask.h
)

add_library(mc_solver SHARED)
target_sources(mc_solver PRIVATE ${mc_solver_SRC})
jrl_target_headers(mc_solver PUBLIC HEADERS ${mc_solver_HDR} BASE_DIRS include)
target_link_libraries(mc_solver PUBLIC mc_rbdyn mc_rtc_gui)
install_mc_rtc_lib(mc_solver)

set(mc_trajectory_SRC mc_trajectory/BSpline.cpp mc_trajectory/InterpolatedRotation.cpp
                      mc_trajectory/spline_utils.cpp mc_trajectory/ExactCubic.cpp
)

set(mc_trajectory_HDR
    include/mc_trajectory/Spline.h
    include/mc_trajectory/Spline.hpp
    include/mc_trajectory/BSpline.h
    include/mc_trajectory/InterpolatedRotation.h
    include/mc_trajectory/spline_utils.h
    include/mc_trajectory/ExactCubic.h
    include/mc_trajectory/SequenceInterpolator.h
    include/mc_trajectory/LinearInterpolation.h
)

add_library(mc_trajectory SHARED)
target_sources(mc_trajectory PRIVATE ${mc_trajectory_SRC})
jrl_target_headers(mc_trajectory PUBLIC HEADERS ${mc_trajectory_HDR} BASE_DIRS include)
target_link_libraries(mc_trajectory PUBLIC ndcurves::ndcurves mc_rtc::mc_rtc_gui)
install_mc_rtc_lib(mc_trajectory)

set(mc_tasks_SRC
    mc_tasks/TrajectoryTaskGeneric.cpp
    mc_tasks/MomentumTask.cpp
    mc_tasks/PositionTask.cpp
    mc_tasks/TransformTask.cpp
    mc_tasks/OrientationTask.cpp
    mc_tasks/VectorOrientationTask.cpp
    mc_tasks/EndEffectorTask.cpp
    mc_tasks/RelativeEndEffectorTask.cpp
    mc_tasks/CoMTask.cpp
    mc_tasks/AddRemoveContactTask.cpp
    mc_tasks/BSplineTrajectoryTask.cpp
    mc_tasks/ExactCubicTrajectoryTask.cpp
    mc_tasks/ComplianceTask.cpp
    mc_tasks/AdmittanceTask.cpp
    mc_tasks/ImpedanceTask.cpp
    mc_tasks/FirstOrderImpedanceTask.cpp
    mc_tasks/DampingTask.cpp
    mc_tasks/CoPTask.cpp
    mc_tasks/GazeTask.cpp
    mc_tasks/PositionBasedVisServoTask.cpp
    mc_tasks/PostureTask.cpp
    mc_tasks/LookAtTask.cpp
    mc_tasks/LookAtFrameTask.cpp
    mc_tasks/MetaTaskLoader.cpp
    mc_tasks/lipm_stabilizer/StabilizerTask.cpp
    mc_tasks/lipm_stabilizer/StabilizerTask_log_gui.cpp
    mc_tasks/lipm_stabilizer/ZMPCC.cpp
    mc_tasks/lipm_stabilizer/Contact.cpp
)

set(mc_tasks_HDR
    include/mc_tasks/AddRemoveContactTask.h
    include/mc_tasks/CoMTask.h
    include/mc_tasks/EndEffectorTask.h
    include/mc_tasks/MetaTaskLoader.h
    include/mc_tasks/MomentumTask.h
    include/mc_tasks/OrientationTask.h
    include/mc_tasks/VectorOrientationTask.h
    include/mc_tasks/PositionTask.h
    include/mc_tasks/SurfaceTransformTask.h
    include/mc_tasks/TransformTask.h
    include/mc_tasks/RelativeEndEffectorTask.h
    include/mc_tasks/SmoothTask.h
    include/mc_tasks/SplineTrajectoryTask.h
    include/mc_tasks/SplineTrajectoryTask.hpp
    include/mc_tasks/BSplineTrajectoryTask.h
    include/mc_tasks/ExactCubicTrajectoryTask.h
    include/mc_tasks/TrajectoryTaskGeneric.h
    include/mc_tasks/ComplianceTask.h
    include/mc_tasks/AdmittanceTask.h
    include/mc_tasks/ImpedanceGains.h
    include/mc_tasks/ImpedanceTask.h
    include/mc_tasks/DampingTask.h
    include/mc_tasks/CoPTask.h
    include/mc_tasks/GazeTask.h
    include/mc_tasks/PositionBasedVisServoTask.h
    include/mc_tasks/PostureTask.h
    include/mc_tasks/LookAtTask.h
    include/mc_tasks/LookAtFrameTask.h
    include/mc_tasks/LookAtSurfaceTask.h
    include/mc_tasks/lipm_stabilizer/StabilizerTask.h
    include/mc_tasks/lipm_stabilizer/Contact.h
    include/mc_tasks/lipm_stabilizer/ZMPCC.h
)

add_library(mc_tasks SHARED)
target_sources(mc_tasks PRIVATE ${mc_tasks_SRC})
jrl_target_headers(mc_tasks PUBLIC HEADERS ${mc_tasks_HDR} BASE_DIRS include)
target_link_libraries(
  mc_tasks
  PUBLIC mc_solver
         mc_rbdyn
         mc_trajectory
         mc_filter
         mc_rtc_utils
         eigen-quadprog::eigen-quadprog
         state-observation::state-observation
)
install_mc_rtc_lib(mc_tasks)

set(mc_planning_SRC)

add_library(mc_planning SHARED)
target_sources(mc_planning PRIVATE mc_planning/Pendulum.cpp)
jrl_target_headers(
  mc_planning PUBLIC HEADERS ${PROJECT_SOURCE_DIR}/include/mc_planning/Pendulum.h
  BASE_DIRS ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(mc_planning PUBLIC mc_rbdyn)
install_mc_rtc_lib(mc_planning)

set(mc_filter_HDR
    include/mc_filter/LowPass.h include/mc_filter/LowPassFiniteDifferences.h
    include/mc_filter/ExponentialMovingAverage.h include/mc_filter/LeakyIntegrator.h
    include/mc_filter/StationaryOffset.h include/mc_filter/utils/clamp.h
)

# header-only
add_library(mc_filter INTERFACE)
jrl_target_headers(mc_filter PUBLIC HEADERS ${mc_filter_HDR} BASE_DIRS include)
target_link_libraries(mc_filter INTERFACE mc_rtc_utils)
install_mc_rtc_header_lib(mc_filter)

set(mc_observers_SRC mc_observers/ObserverLoader.cpp mc_observers/Observer.cpp
                     mc_observers/ObserverPipeline.cpp
)

set(mc_observers_HDR
    include/mc_observers/ObserverLoader.h include/mc_observers/Observer.h
    include/mc_observers/ObserverPipeline.h include/mc_observers/ObserverMacros.h
)

set(mc_control_SRC
    mc_control/CompletionCriteria.cpp
    mc_control/ControllerServer.cpp
    mc_control/ControllerServerConfiguration.cpp
    mc_control/SimulationContactPair.cpp
    mc_control/MCController.cpp
    mc_control/mc_python_controller.cpp
    mc_control/mc_global_controller.cpp
    mc_control/mc_global_controller_configuration.cpp
    mc_control/mc_global_controller_gui.cpp
    mc_control/mc_global_controller_services.cpp
    mc_solver/QPSolver_setContacts.cpp
    mc_control/Ticker.cpp
)

if(MC_RTC_BUILD_STATIC)
  list(APPEND mc_control_SRC mc_control/ControllerLoader.cpp)
  list(APPEND mc_control_SRC mc_control/GlobalPluginLoader.cpp)
endif()

set(mc_control_HDR
    include/mc_control/CompletionCriteria.h
    include/mc_control/Configuration.h
    include/mc_control/Contact.h
    include/mc_control/ControllerServer.h
    include/mc_control/ControllerServerConfiguration.h
    include/mc_control/GlobalPlugin.h
    include/mc_control/GlobalPluginMacros.h
    include/mc_control/GlobalPlugin_fwd.h
    include/mc_control/MCController.h
    include/mc_control/Ticker.h
    include/mc_control/mc_controller.h
    include/mc_control/mc_python_controller.h
    include/mc_control/SimulationContactPair.h
    include/mc_control/mc_global_controller.h
)

add_library(mc_control SHARED)
target_sources(mc_control PRIVATE ${mc_control_SRC} ${mc_observers_SRC})
jrl_target_headers(mc_control PUBLIC HEADERS ${mc_control_HDR} BASE_DIRS include)
jrl_target_headers(mc_control PUBLIC HEADERS ${mc_observers_HDR} BASE_DIRS include)
target_link_libraries(
  mc_control PUBLIC mc_tasks mc_solver mc_rtc_loader mc_rtc_utils mc_rtc_gui
)
if(NOT MC_RTC_DISABLE_NETWORK)
  target_link_libraries(mc_control PUBLIC nanomsg)
else()
  target_compile_definitions(mc_control PUBLIC MC_RTC_DISABLE_NETWORK)
endif()
install_mc_rtc_lib(mc_control)

add_library(mc_observers ALIAS mc_control)
add_library(mc_rtc::mc_observers ALIAS mc_control)
add_subdirectory(mc_observers)

set(mc_control_client_SRC mc_control/ControllerClient.cpp)

set(mc_control_client_HDR include/mc_control/ControllerClient.h)

add_library(mc_control_client SHARED)
target_sources(mc_control_client PRIVATE ${mc_control_client_SRC})
jrl_target_headers(
  mc_control_client PUBLIC HEADERS ${mc_control_client_HDR} BASE_DIRS include
)
target_link_libraries(mc_control_client PUBLIC mc_control)
install_mc_rtc_lib(mc_control_client)

add_subdirectory(mc_control)
