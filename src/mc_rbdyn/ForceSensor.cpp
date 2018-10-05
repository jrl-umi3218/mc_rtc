#include <mc_rbdyn/ForceSensor.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/logging.h>

#include <fstream>

namespace mc_rbdyn
{

namespace detail
{
/** Structure used to hold the calibration data. This structure is *NOT*
 * intended to be constructed directly, see @ForceSensorsCalibrator for usage.
 * It is composed of 13 parameters (real numbers) that should be loaded
 * from a calibration file. **/
struct ForceSensorCalibData
{
public:
  /** Default constructor, always returns zero contribution */
  ForceSensorCalibData() {}

  /** Load data from a file, using a gravity vector. The file
   * should contain 13 parameters in that order: mass (1),
   * rpy for X_f_ds (3), position for X_p_vb (3), wrench
   * offset (6).
   * @return Default calibration if the file does not exist,
   * throws if the file is ill-formed. */
  ForceSensorCalibData(const std::string & filename, const Eigen::Vector3d & gravity) : ForceSensorCalibData()
  {
    loadData(filename, gravity);
  }

  void loadData(const std::string & filename, const Eigen::Vector3d & gravity)
  {
    const int nr_params = 13;
    std::ifstream strm(filename);
    if(!strm.is_open())
    {
      LOG_ERROR("Could not open " << filename)
      return;
    }

    // Vector 13d
    Eigen::Matrix<double, nr_params, 1> X;
    double temp;
    for(int i = 0; i < nr_params; ++i)
    {
      strm >> temp;
      if(!strm.good())
      {
        LOG_ERROR("Invalid calibration file")
        return;
      }
      if(strm.eof())
      {
        LOG_ERROR("File too short, should have " << nr_params << " parameters")
        return;
      }
      X(i) = temp;
    }

    mass_ = X(0);
    worldForce_ = sva::ForceVecd(Eigen::Vector3d(0., 0., 0.), -X(0) * gravity);
    X_f_ds_ = mc_rbdyn::rpyToPT(X.segment<3>(1));
    X_p_vb_ = sva::PTransformd(Eigen::Matrix3d::Identity(), X.segment<3>(4));
    offset_ = sva::ForceVecd(X.segment<6>(7));
  }

  /** Return the gravity wrench applied on the force sensor in the sensor
   * frame, i.e. the external force $f_{ext}$ is:
   * $f_{ext} = f_{mes} - wfToSensor$.
   * @param X_0_p the transform in the world frame of the parent body of the sensor
   * @param X_p_f the transform from the parent body to the sensor itself
   */
  inline sva::ForceVecd wfToSensor(const sva::PTransformd & X_0_p, const sva::PTransformd & X_p_f) const
  {
    sva::PTransformd X_0_ds = X_f_ds_ * X_p_f * X_0_p;
    sva::PTransformd X_0_vb(X_0_ds.inv().rotation(), (X_p_vb_ * X_0_p * X_0_ds.inv()).translation());
    return offset_ + X_0_vb.transMul(worldForce_);
  }

  /** Return X_f_ds, the pure rotation transform from the position given by
   * the URDF and the one estimated by calibration
   */
  inline const sva::PTransformd & X_f_ds() const
  {
    return X_f_ds_;
  }

  /** Return the mass of the link generating the wrench */
  inline double mass() const
  {
    return mass_;
  }

  inline const sva::ForceVecd & offset() const
  {
    return offset_;
  }

private:
  /** Mass of the link generating the wrench */
  double mass_ = 0.0;
  /** Constant gravity wrench applied on the force sensor
   * in the world frame: $\{0, mg \} */
  sva::ForceVecd worldForce_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  /** Local rotation between the model force sensor and real one */
  sva::PTransformd X_f_ds_ = sva::PTransformd::Identity();
  /** Transform from the parent body to the virtual link CoM */
  sva::PTransformd X_p_vb_ = sva::PTransformd::Identity();
  /** Force/torque offset */
  sva::ForceVecd offset_ = sva::ForceVecd(Eigen::Vector6d::Zero());
};
} // namespace detail

ForceSensor::ForceSensor() : ForceSensor("", "", sva::PTransformd::Identity()) {}

ForceSensor::ForceSensor(const std::string & name, const std::string & parentBodyName, const sva::PTransformd & X_p_f)
: name_(name), parentBody_(parentBodyName), X_p_f_(X_p_f), wrench_(Eigen::Vector6d::Zero()),
  calibration_(std::make_shared<detail::ForceSensorCalibData>())
{
}

ForceSensor::~ForceSensor() {}

const std::string & ForceSensor::name() const
{
  return name_;
}

const std::string & ForceSensor::parentBody() const
{
  return parentBody_;
}

const sva::PTransformd & ForceSensor::X_p_f() const
{
  return X_p_f_;
}

const sva::ForceVecd & ForceSensor::wrench() const
{
  return wrench_;
}

void ForceSensor::wrench(const sva::ForceVecd & wrench)
{
  wrench_ = wrench;
}

void ForceSensor::loadCalibrator(const std::string & calib_file, const Eigen::Vector3d & gravity)
{
  calibration_->loadData(calib_file, gravity);
}

const sva::PTransformd & ForceSensor::X_fsmodel_fsactual() const
{
  return calibration_->X_f_ds();
}

const sva::PTransformd ForceSensor::X_fsactual_parent() const
{
  return (X_fsmodel_fsactual() * X_p_f_).inv();
}

double ForceSensor::mass() const
{
  return calibration_->mass();
}

const sva::ForceVecd & ForceSensor::offset() const
{
  return calibration_->offset();
}

const sva::ForceVecd ForceSensor::wrenchWithoutGravity(const mc_rbdyn::Robot & robot) const
{
  sva::PTransformd X_0_p = robot.mbc().bodyPosW[robot.bodyIndexByName(parentBody_)];
  auto w = wrench_ - calibration_->wfToSensor(X_0_p, X_p_f_);
  return w;
}

sva::ForceVecd ForceSensor::worldWrench(const mc_rbdyn::Robot & robot) const
{
  sva::ForceVecd w_fsactual = wrench();
  sva::PTransformd X_parent_0 = robot.mbc().bodyPosW[robot.bodyIndexByName(parentBody_)].inv();
  sva::PTransformd X_fsactual_0 = X_parent_0 * X_fsactual_parent();
  return X_fsactual_0.dualMul(w_fsactual);
}

sva::ForceVecd ForceSensor::worldWrenchWithoutGravity(const mc_rbdyn::Robot & robot) const
{
  sva::ForceVecd w_fsactual = wrenchWithoutGravity(robot);
  sva::PTransformd X_parent_0 = robot.mbc().bodyPosW[robot.bodyIndexByName(parentBody_)].inv();
  sva::PTransformd X_fsactual_0 = X_parent_0 * X_fsactual_parent();
  return X_fsactual_0.dualMul(w_fsactual);
}

} // namespace mc_rbdyn
