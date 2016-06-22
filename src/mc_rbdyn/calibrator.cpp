#include <mc_rbdyn/calibrator.h>
#include <mc_rbdyn/rpy_utils.h>

#include <boost/filesystem/fstream.hpp>

#include <mc_rtc/logging.h>

namespace mc_rbdyn
{

ForceSensorsCalibrator::CalibData::CalibData()
  : worldForce_(Eigen::Vector6d::Zero()),
    X_f_ds_(sva::PTransformd::Identity()),
    X_p_vb_(sva::PTransformd::Identity()),
    offset_(Eigen::Vector6d::Zero())
  {}

ForceSensorsCalibrator::CalibData::CalibData(const boost::filesystem::path& filename,
    const Eigen::Vector3d& gravity)
  : CalibData()
{
  const int nr_params = 13;
  boost::filesystem::ifstream strm(filename);
  if(!strm.is_open())
  {
    LOG_ERROR("Could not open " << filename)
    return;
  }

  //Vector 13d
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
  worldForce_ = sva::ForceVecd(Eigen::Vector3d(0., 0., 0.),
                               -X(0)*gravity);
  X_f_ds_ = mc_rbdyn::rpyToPT(X.segment<3>(1));
  X_p_vb_ = sva::PTransformd(Eigen::Matrix3d::Identity(), X.segment<3>(4));
  offset_ = sva::ForceVecd(X.segment<6>(7));
}

sva::ForceVecd ForceSensorsCalibrator::CalibData::wfToSensor(const sva::PTransformd& X_0_p,
                                     const sva::PTransformd& X_p_f) const
{
  sva::PTransformd X_0_ds = X_f_ds_*X_p_f*X_0_p;
  sva::PTransformd X_0_vb(X_0_ds.inv().rotation(),
                          (X_p_vb_*X_0_p*X_0_ds.inv()).translation());
  return offset_ + X_0_vb.transMul(worldForce_);
}

ForceSensorsCalibrator::ForceSensorsCalibrator()
  : sensors_(),
    calibrations_()
{}

ForceSensorsCalibrator::ForceSensorsCalibrator(std::shared_ptr<mc_rbdyn::RobotModule> robot_module)
{
  //Provide default gravity here because it may be on Y-axis
  Eigen::Vector3d gravity(0., 0., 9.81);
  for(const auto & sensor : robot_module->forceSensors())
  {
    sensors_[sensor.sensorName] = sensor;
    boost::filesystem::path path(robot_module->calib_dir);
    auto filename = path /= "calib_data." + sensor.sensorName;
    calibrations_[sensor.sensorName] = CalibData(filename, gravity);
  }
}

void ForceSensorsCalibrator::removeGravityFromAll(std::map<std::string, sva::ForceVecd>& wrenches,
                                      const mc_rbdyn::Robot& robot) const
{
  for(auto & pair : wrenches)
  {
    removeGravity(pair.second, pair.first, robot);
  }
}

void ForceSensorsCalibrator::removeGravity(sva::ForceVecd& wrench,
                               const std::string& name, const mc_rbdyn::Robot& robot) const
{
  sva::PTransformd X_0_p = robot.mbc().bodyPosW[robot.bodyIndexByName(sensors_.at(name).parentBodyName)];
  wrench -= calibrations_.at(name).wfToSensor(X_0_p, sensors_.at(name).X_p_f);
}

}
