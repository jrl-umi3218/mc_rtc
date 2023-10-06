/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/ForceSensor.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>
#include <fstream>
namespace bfs = boost::filesystem;

namespace mc_rbdyn
{

namespace detail
{

static std::string CalibDataRequirements =
    R"(The file should contain 13 parameters in that order:
    - mass (1)
    - rpy of local rotation between the model force sensor and real one (3)
    - translation from the parent body to the virtual link CoM (3)
    - wrench offset (6).)";

void ForceSensorCalibData::reset()
{
  mass = 0.0;
  worldForce = sva::ForceVecd(Eigen::Vector6d::Zero());
  X_f_ds = sva::PTransformd::Identity();
  X_p_vb = sva::PTransformd::Identity();
  offset = sva::ForceVecd(Eigen::Vector6d::Zero());
}

void ForceSensorCalibData::loadData(const std::string & filename, const Eigen::Vector3d & gravity)
{
  constexpr int nr_params = 13;
  std::ifstream strm(filename);
  if(!strm.is_open())
  {
    mc_rtc::log::error("Could not open {}", filename);
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
      mc_rtc::log::error_and_throw<std::invalid_argument>("[ForceSensorCalibData] Invalid calibration file {}. {}",
                                                          filename, CalibDataRequirements);
    }
    if(strm.eof())
    {
      mc_rtc::log::error_and_throw<std::invalid_argument>("[ForceSensorCalibData] Calibration file {} is too short. {}",
                                                          filename, CalibDataRequirements);
    }
    X(i) = temp;
  }

  mass = X(0);
  worldForce = sva::ForceVecd(Eigen::Vector3d(0., 0., 0.), -X(0) * gravity);
  X_f_ds = mc_rbdyn::rpyToPT(X.segment<3>(1));
  X_p_vb = sva::PTransformd(Eigen::Matrix3d::Identity(), X.segment<3>(4));
  offset = sva::ForceVecd(X.segment<6>(7));
}

/** Return the gravity wrench applied on the force sensor in the sensor
 * frame, i.e. the external force $f_{ext}$ is:
 * $f_{ext} = f_{mes} - wfToSensor$.
 * @param X_0_p the transform in the world frame of the parent body of the sensor
 * @param X_p_f the transform from the parent body to the sensor itself
 */
sva::ForceVecd ForceSensorCalibData::wfToSensor(const sva::PTransformd & X_0_p,
                                                const sva::PTransformd & X_p_f) const noexcept
{
  sva::PTransformd X_0_ds = X_f_ds * X_p_f * X_0_p;
  sva::PTransformd X_0_vb(X_0_ds.inv().rotation(), (X_p_vb * X_0_p * X_0_ds.inv()).translation());
  return offset + X_0_vb.transMul(worldForce);
}

} // namespace detail

ForceSensor::ForceSensor() : ForceSensor("", "", sva::PTransformd::Identity()) {}

ForceSensor::ForceSensor(const std::string & name, const std::string & parentBodyName, const sva::PTransformd & X_p_f)
: Device(name, parentBodyName, X_p_f), wrench_(Eigen::Vector6d::Zero())
{
  type_ = "ForceSensor";
}

ForceSensor::ForceSensor(const ForceSensor & fs) : ForceSensor(fs.name_, fs.parentBody(), fs.X_p_f())
{
  wrench_ = fs.wrench_;
  calibration_ = fs.calibration_;
}

ForceSensor & ForceSensor::operator=(const ForceSensor & fs)
{
  if(&fs == this) { return *this; }
  name_ = fs.name_;
  parent_ = fs.parent_;
  X_p_s_ = fs.X_p_s_;
  calibration_ = fs.calibration_;
  wrench_ = fs.wrench_;
  return *this;
}

ForceSensor::~ForceSensor() noexcept = default;

void ForceSensor::loadCalibrator(const detail::ForceSensorCalibData & data) noexcept
{
  calibration_ = data;
}

void ForceSensor::loadCalibrator(const std::string & calib_file, const Eigen::Vector3d & gravity)
{
  if(!bfs::exists(calib_file))
  {
    mc_rtc::log::warning("No calibration file {} found for force sensor {}", calib_file, name());
    return;
  }
  calibration_.loadData(calib_file, gravity);
}

void ForceSensor::copyCalibrator(const mc_rbdyn::ForceSensor & other)
{
  calibration_ = other.calibration_;
}

void ForceSensor::resetCalibrator()
{
  calibration_.reset();
}

const sva::PTransformd & ForceSensor::X_fsmodel_fsactual() const
{
  return calibration_.X_f_ds;
}

const sva::PTransformd ForceSensor::X_fsactual_parent() const
{
  return (X_fsmodel_fsactual() * X_p_s_).inv();
}

double ForceSensor::mass() const
{
  return calibration_.mass;
}

const sva::ForceVecd & ForceSensor::offset() const
{
  return calibration_.offset;
}

sva::ForceVecd ForceSensor::wrenchWithoutGravity(const mc_rbdyn::Robot & robot) const
{
  sva::PTransformd X_0_p = robot.mbc().bodyPosW[robot.bodyIndexByName(parent_)];
  auto w = wrench_ - calibration_.wfToSensor(X_0_p, X_p_s_);
  return w;
}

sva::ForceVecd ForceSensor::worldWrench(const mc_rbdyn::Robot & robot) const
{
  sva::ForceVecd w_fsactual = wrench();
  sva::PTransformd X_parent_0 = robot.mbc().bodyPosW[robot.bodyIndexByName(parent_)].inv();
  sva::PTransformd X_fsactual_0 = X_parent_0 * X_fsactual_parent();
  return X_fsactual_0.dualMul(w_fsactual);
}

sva::ForceVecd ForceSensor::worldWrenchWithoutGravity(const mc_rbdyn::Robot & robot) const
{
  sva::ForceVecd w_fsactual = wrenchWithoutGravity(robot);
  sva::PTransformd X_parent_0 = robot.mbc().bodyPosW[robot.bodyIndexByName(parent_)].inv();
  sva::PTransformd X_fsactual_0 = X_parent_0 * X_fsactual_parent();
  return X_fsactual_0.dualMul(w_fsactual);
}

DevicePtr ForceSensor::clone() const
{
  return DevicePtr(new ForceSensor(*this));
}

} // namespace mc_rbdyn
