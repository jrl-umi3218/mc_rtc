/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Device.h>
#include <mc/rtc/deprecated.hh>

#include <Eigen/StdVector>

namespace mc_rbdyn
{

/** This structure defines a body sensor, that is a sensor that provides
 * dynamic information about a body. It would typically be used to represent an
 * IMU reading but in more ideal (simulation, external tracking system...) it
 * can hold more information */
struct MC_RBDYN_DLLAPI BodySensor : public Device
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** Default constructor, does not represent a valid body sensor */
  inline BodySensor() : BodySensor("", "", sva::PTransformd::Identity()) {}

  /** Constructor
   *
   * @param name Name of the sensor
   *
   * @param bodyName Name of the body to which the sensor is attached
   *
   * @param X_b_s Transformation from the parent body to the sensor
   *
   */
  inline BodySensor(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s)
  : Device(name, bodyName, X_b_s)
  {
    type_ = "BodySensor";
  }

  BodySensor(const BodySensor & bs) : BodySensor(bs.name(), bs.parent(), bs.X_b_s())
  {
    position_ = bs.position_;
    orientation_ = bs.orientation_;
    linear_velocity_ = bs.linear_velocity_;
    angular_velocity_ = bs.angular_velocity_;
    linear_acceleration_ = bs.linear_acceleration_;
    angular_acceleration_ = bs.angular_acceleration_;
  }

  BodySensor & operator=(const BodySensor & bs)
  {
    name_ = bs.name_;
    parent_ = bs.parent_;
    X_p_s_ = bs.X_p_s_;
    position_ = bs.position_;
    orientation_ = bs.orientation_;
    linear_velocity_ = bs.linear_velocity_;
    angular_velocity_ = bs.angular_velocity_;
    linear_acceleration_ = bs.linear_acceleration_;
    angular_acceleration_ = bs.angular_acceleration_;
    return *this;
  }

  BodySensor(BodySensor &&) = default;
  BodySensor & operator=(BodySensor &&) = default;

  ~BodySensor() noexcept override;

  /** Get the sensor's parent body name */
  inline const std::string & parentBody() const
  {
    return Device::parent();
  }

  /** Return the transformation from the parent body to the sensor */
  inline const sva::PTransformd & X_b_s() const
  {
    return Device::X_p_s();
  }

  /** Return the sensor's position reading, Zero if not provided */
  inline const Eigen::Vector3d & position() const
  {
    return position_;
  }

  /** Set the sensor's position reading */
  inline void position(const Eigen::Vector3d & position)
  {
    position_ = position;
  }

  /** Return the sensor's orientation reading, Identity if not provided */
  inline const Eigen::Quaterniond & orientation() const
  {
    return orientation_;
  }

  /** Set the sensor's orientation reading
   *
   * \note By convention, this rotation should be given from the inertial frame
   * (i.e. a fixed frame in the real world) to a body frame of the robot. For
   * instance, on HRP-4 the body sensor orientation goes from the inertial
   * frame to the "base_link" frame.
   *
   */
  inline void orientation(const Eigen::Quaterniond & orientation)
  {
    orientation_ = orientation;
  }

  /** Return the sensor's linear velocity reading, Zero if not provided */
  inline const Eigen::Vector3d & linearVelocity() const
  {
    return linear_velocity_;
  }

  /** Set the sensor's linear velocity reading */
  inline void linearVelocity(const Eigen::Vector3d & linear_velocity)
  {
    linear_velocity_ = linear_velocity;
  }

  /** Return the sensor's angular velocity reading, Zero if not provided */
  inline const Eigen::Vector3d & angularVelocity() const
  {
    return angular_velocity_;
  }

  /** Set the sensor's angular velocity reading */
  inline void angularVelocity(const Eigen::Vector3d & angular_velocity)
  {
    angular_velocity_ = angular_velocity;
  }

  /** Return the sensor's linear acceleration reading, Zero if not provided
   *
   * @deprecated in favor of const Eigen::Vector3d & linearAcceleration() const
   **/
  MC_RTC_DEPRECATED inline const Eigen::Vector3d & acceleration() const
  {
    return linearAcceleration();
  }

  /** Set the sensor's linear acceleration reading
   *
   * @deprecated in favor of void linearAcceleration(const Eigen::Vector3d &)
   * */
  MC_RTC_DEPRECATED inline void acceleration(const Eigen::Vector3d & acceleration)
  {
    linearAcceleration(acceleration);
  }

  /** Return the sensor's linear acceleration reading, Zero if not provided */
  inline const Eigen::Vector3d & linearAcceleration() const
  {
    return linear_acceleration_;
  }

  /** Set the sensor's linear acceleration reading */
  inline void linearAcceleration(const Eigen::Vector3d & linear_acceleration)
  {
    linear_acceleration_ = linear_acceleration;
  }

  /** Return the sensor's angular acceleration reading, Zero if not provided */
  inline const Eigen::Vector3d & angularAcceleration() const
  {
    return angular_acceleration_;
  }

  /** Set the sensor's angular acceleration reading */
  inline void angularAcceleration(const Eigen::Vector3d & angular_acceleration)
  {
    angular_acceleration_ = angular_acceleration;
  }

  DevicePtr clone() const override;

private:
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_acceleration_ = Eigen::Vector3d::Zero();
};

typedef std::vector<BodySensor, Eigen::aligned_allocator<BodySensor>> BodySensorVector;

} // namespace mc_rbdyn
