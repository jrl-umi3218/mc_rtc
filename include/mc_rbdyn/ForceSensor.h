/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Device.h>

namespace mc_rbdyn
{

struct Robot;

namespace detail
{
struct ForceSensorCalibData;
}

/** This struct is intended to hold static information about a force sensor
 * and the current reading of said sensor. If the appropriate data is
 * provided, a gravity-free reading can be provided.
 */
struct MC_RBDYN_DLLAPI ForceSensor : public Device
{
public:
  /** Default constructor, this does not represent a valid force sensor */
  ForceSensor();

  /** Construct a valid force sensor based on static information, this
   * force sensor can then be used to provide sensing information to the
   * robot. However, filtering will have no effect
   *
   * @param name Name of the sensor
   *
   * @param parentBodyName Name of the sensor's parent body
   *
   * @param X_p_f Model transformation from the parent body to the model (not
   * real) sensor frame
   *
   */
  ForceSensor(const std::string & name, const std::string & parentBodyName, const sva::PTransformd & X_p_f);

  ForceSensor(const ForceSensor & fs);

  ForceSensor & operator=(const ForceSensor & fs);

  ForceSensor(ForceSensor &&) = default;
  ForceSensor & operator=(ForceSensor &&) = default;

  /** Destructor */
  ~ForceSensor() noexcept override;

  /** Return the sensor's parent body */
  inline const std::string & parentBody() const
  {
    return Device::parent();
  }

  /** Return the transformation from the parent body to the sensor (model) */
  inline const sva::PTransformd & X_p_f() const
  {
    return Device::X_p_s();
  }

  /** Return the sensor pose in the inertial frame (convenience function) */
  inline sva::PTransformd X_0_f(const mc_rbdyn::Robot & robot) const
  {
    return Device::X_0_s(robot);
  }

  /** Return the current wrench */
  inline const sva::ForceVecd & wrench() const
  {
    return wrench_;
  }

  /** Return the force reading
   *
   * Shortcut for wrench().force()
   */
  inline const Eigen::Vector3d & force() const
  {
    return wrench().force();
  }

  /** Return the couple reading
   *
   * Shortcut for wrench().couple()
   */
  inline const Eigen::Vector3d & couple() const
  {
    return wrench().couple();
  }

  /** Set the current wrench expressed in sensor frame
   *
   * @param wrench New wrench reading
   */
  inline void wrench(const sva::ForceVecd & wrench)
  {
    wrench_ = wrench;
  }

  /** Return a gravity-free wrench in sensor frame
   *
   * @param robot Robot that the sensor belongs to
   *
   * @returns A gravity-free reading of the wrench
   *
   */
  sva::ForceVecd wrenchWithoutGravity(const mc_rbdyn::Robot & robot) const;

  /** Return measured wrench in the inertial frame
   *
   * @param robot Robot that the sensor belongs to
   *
   * @returns w_0 Spatial force vector of measured wrench
   *
   */
  sva::ForceVecd worldWrench(const mc_rbdyn::Robot & robot) const;

  /** Return measured gravity-free wrench in the inertial frame
   *
   * @param robot Robot that the sensor belongs to
   *
   * @returns w_0 Spatial force vector of measured wrench
   *
   */
  sva::ForceVecd worldWrenchWithoutGravity(const mc_rbdyn::Robot & robot) const;

  /** @name Calibration
   *
   * These functions are related to the sensor's calibration
   *
   * @{
   */

  /** Load data from a file, using a gravity vector. The file should
   * contain 13 parameters in that order: mass (1), rpy for X_f_ds (3),
   * position for X_p_vb (3), wrench offset (6).
   *
   * @param calib_file Calibration file, the file should exist
   *
   * @param gravity Gravity vector, defaults to Z
   *
   */
  void loadCalibrator(const std::string & calib_file, const Eigen::Vector3d & gravity = {0., 0., 9.81});

  /**
   * Reset the force calibration to its default values such that the calibrator
   * has no effect on the sensor wrench
   **/
  void resetCalibrator();

  /** Return the local rotation associated to the sensor, i.e. the error
   * between the model forceSensor and real one
   */
  const sva::PTransformd & X_fsmodel_fsactual() const;

  /** Return the transform from the parent body to the real force sensor
   */
  const sva::PTransformd X_fsactual_parent() const;

  /** Return the mass of the sensor */
  double mass() const;

  /** Return the sensor offset */
  const sva::ForceVecd & offset() const;

  /** @} */
  /* End of Calibration group */

  DevicePtr clone() const override;

private:
  sva::ForceVecd wrench_;

  std::shared_ptr<detail::ForceSensorCalibData> calibration_;
};

} // namespace mc_rbdyn
