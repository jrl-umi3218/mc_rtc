#pragma once

#include <mc_rtc/Schema.h>

namespace mc_rbdyn::detail
{

struct ForceSensorCalibData
{
  MC_RTC_NEW_SCHEMA(ForceSensorCalibData)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(ForceSensorCalibData, __VA_ARGS__))
  MEMBER(double, mass, "Mass of the link generating the wrench")
  MEMBER(sva::ForceVecd, worldForce, "Constant gravity wrench applied on the force sensor in the world frame")
  MEMBER(sva::PTransformd, X_f_ds, "Local rotation between the model force sensor and the real one")
  MEMBER(sva::PTransformd, X_p_vb, "Transform from the parent body to the virtual link CoM")
  MEMBER(sva::ForceVecd, offset, "Force/torque offset")
#undef MEMBER

  /** Restore the calibrator default values such that it always returns zero
   * contribution
   */
  void reset();

  /** Load data from a file, using a gravity vector. The file
   * should contain 13 parameters in that order: mass (1),
   * rpy for X_f_ds (3), position for X_p_vb (3), wrench
   * offset (6).
   *
   * If the file does not exist, default calibration parameters that do nothing
   * will be used. If the file exists but its parameters are invalid, an exception will be thrown.
   *
   * @throws std::invalid_argument if the file is ill-formed.
   **/
  void loadData(const std::string & filename, const Eigen::Vector3d & gravity);

  /** Return the gravity wrench applied on the force sensor in the sensor
   * frame, i.e. the external force $f_{ext}$ is:
   * $f_{ext} = f_{mes} - wfToSensor$.
   * @param X_0_p the transform in the world frame of the parent body of the sensor
   * @param X_p_f the transform from the parent body to the sensor itself
   */
  sva::ForceVecd wfToSensor(const sva::PTransformd & X_0_p, const sva::PTransformd & X_p_f) const noexcept;
};

} // namespace mc_rbdyn::detail
