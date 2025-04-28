#include <mc_rbdyn/ForceSensor.h>
#include <mc_rbdyn/ForceSensorCalibData.h>
#include <mc_rbdyn/Robot.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_ForceSensorCalibData(nb::module_ & m)
{
  using FD = mc_rbdyn::detail::ForceSensorCalibData;
  auto fd = nb::class_<mc_rbdyn::detail::ForceSensorCalibData>(m, "ForceSensorCalibData");
  fd.def(nb::init());

  fd.def("reset", &FD::reset,
         R"(
   Restore the calibrator default values such that it always returns zero contribution
                                  )")
      .def("loadData", &FD::loadData, "filename"_a, "gravity"_a,
           R"(
   Load data from a file, using a gravity vector. The file
   should contain 13 parameters in that order: mass (1),
   rpy for X_f_ds (3), position for X_p_vb (3), wrench
   offset (6).

   If the file does not exist, default calibration parameters that do nothing will be used. If the file exists but its parameters are invalid, an exception will be thrown.

   :throws: if the file is ill-formed.
   )")
      .def("wfToSensor", &FD::wfToSensor, "X_0_p"_a, "X_p_f"_a,
           R"(
  Return the gravity wrench applied on the force sensor in the sensor
  frame, i.e. the external force $f_{ext}$ is:
  $f_{ext} = f_{mes} - wfToSensor$.
  :param X_0_p: the transform in the world frame of the parent body of the sensor
  :param X_p_f: the transform from the parent body to the sensor itself
                                                  )");

  // bind schema parameters
  // XXX: could we do that automagically?
  fd.def_rw("mass", &FD::mass, "Mass of the link generating the wrench");
  fd.def_rw("worldForce", &FD::worldForce, "Constant gravity wrench applied on the force sensor in the world frame");
  fd.def_rw("X_f_ds", &FD::X_f_ds, "Local rotation between the model force and the real one");
  fd.def_rw("X_p_vb", &FD::X_p_vb, "Transform from the parent body to the virtual link CoM");
  fd.def_rw("offset", &FD::offset, "Force/torque offset");
}

void bind_ForceSensor(nb::module_ & m)
{
  using FS = mc_rbdyn::ForceSensor;
  auto fs = nb::class_<mc_rbdyn::ForceSensor, mc_rbdyn::Device>(m, "ForceSensor");

  fs.doc() = R"(
This class is intended to hold static information about a force sensor
and the current reading of said sensor. If the appropriate data is
provided, a gravity-free reading can be provided.
)";

  fs.def(nb::init(), "Default constructor, this does not represent a valid force sensor")
      .def(nb::init<const std::string &, const std::string &, const sva::PTransformd &>(), "name"_a, "parentBodyName"_a,
           "X_p_f"_a,
           R"(
Construct a valid force sensor based on static information, this
force sensor can then be used to provide sensing information to the
robot. However, filtering will have no effect

:param name: Name of the sensor
:param parentBodyName: Name of the sensor's parent body
:param X_p_f: Model transformation from the parent body to the model (not real) sensor frame
                    )");

  fs.def("parentBody", &FS::parentBody, "Return the sensor's parent body")
      .def("X_p_f", &FS::X_p_f, "Return the transformation from the parent body to the sensor (model)")
      .def("X_0_f", &FS::X_0_f, "Return the sensor pose in the inertial frame (convenience function)");

  // TODO: finish binding ForceSensor
  fs.def_prop_rw(
      "wrench", [](FS & self) -> const sva::ForceVecd { return self.wrench(); },
      [](FS & self, const sva::ForceVecd & wrench) { self.wrench(wrench); }, "current wrench");
  fs.def("force", &FS::force,
         R"(
                  Return the force reading

                  Shortcut for :py:func:`wrench().force()`
                  )");
  fs.def("couple", &FS::couple,
         R"(
                  Return the couple reading

                  Shortcut for :py:func:`wrench().couple()`
                  )");
  fs.def("wrenchWithoutGravity", &FS::wrenchWithoutGravity, "robot"_a,
         R"(
   Return a gravity-free wrench in sensor frame

  :param robot: Robot that the sensor belongs to

  :returns: A gravity-free reading of the wrench
                  )");
  fs.def("worldWrenchW", &FS::worldWrench, "robot"_a,
         R"(
   Return measured wrench in the inertial frame

  :param robot: Robot that the sensor belongs to
  :returns w_0: Spatial force vector of measured wrench
                  )");
  fs.def("worldWrenchWithoutGravity", &FS::worldWrenchWithoutGravity, "robot"_a,
         R"(
   Return measured gravity-free wrench in the inertial frame

  :param robot: Robot that the sensor belongs to

  :returns w_0: Spatial force vector of measured wrench
                  )");
  fs.def("loadCalibrator", nb::overload_cast<const mc_rbdyn::detail::ForceSensorCalibData &>(&FS::loadCalibrator),
         "data"_a,
         R"(
                  Load calibration data
                  )");
  fs.def("loadCalibrator", nb::overload_cast<const std::string &, const Eigen::Vector3d &>(&FS::loadCalibrator),
         "calib_file"_a, "gravity"_a,
         R"(
  Load data from a file, using a gravity vector. The file should
  contain 13 parameters in that order: mass (1), rpy for X_f_ds (3),
  position for X_p_vb (3), wrench offset (6).

  :param calib_file: Calibration file, the file should exist
  :param gravity: Gravity vector, defaults to Z
                  )");
  fs.def("copyCalibrator", &FS::copyCalibrator, "other"_a,
         R"(
   Copy the calibration data from another force sensor

  :param other: Other force sensor from which the data is copied
                  )");
  fs.def("resetCalibrator", &FS::resetCalibrator,
         R"(
   Reset the force calibration to its default values such that the calibrator
   has no effect on the sensor wrench
                  )");
  fs.def("X_fsmodel_fsactual", &FS::X_fsmodel_fsactual,
         R"(
  Return the local rotation associated to the sensor, i.e. the error
  between the model forceSensor and real one
                  )");
  fs.def("X_fsactual_parent", &FS::X_fsactual_parent,
         R"(
  Return the transform from the parent body to the real force sensor
                  )");
  fs.def("mass", &FS::mass, "Return the mass of the sensor")
      .def("offset", &FS::offset, "Return the sensor offset")
      .def("calib", &FS::calib, "Access the calibration object");
  fs.def("clone", &FS::clone);
  fs.def(nb::self == nb::self);
}

} // namespace mc_rtc_python
