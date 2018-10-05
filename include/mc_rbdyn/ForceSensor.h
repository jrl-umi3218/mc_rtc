#pragma once

#include <mc_rbdyn/api.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <memory>

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
struct MC_RBDYN_DLLAPI ForceSensor
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

  /** Destructor */
  ~ForceSensor();

  /** Return the name of the sensor */
  const std::string & name() const;

  /** Return the sensor's parent body */
  const std::string & parentBody() const;

  /** Return the transformation from the parent body to the sensor (model) */
  const sva::PTransformd & X_p_f() const;

  /** Return the current wrench */
  const sva::ForceVecd & wrench() const;

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
  void wrench(const sva::ForceVecd & wrench);

  /** Return a gravity-free wrench in sensor frame
   *
   * @param robot Robot that the sensor belongs to
   *
   * @returns A gravity-free reading of the wrench
   *
   */
  const sva::ForceVecd wrenchWithoutGravity(const mc_rbdyn::Robot & robot) const;

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
private:
  std::string name_;
  std::string parentBody_;
  sva::PTransformd X_p_f_;

  sva::ForceVecd wrench_;

  std::shared_ptr<detail::ForceSensorCalibData> calibration_;
};

} // namespace mc_rbdyn
