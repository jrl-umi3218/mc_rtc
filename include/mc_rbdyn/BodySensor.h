#pragma once

#include <mc_rbdyn/api.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/StdVector>

namespace mc_rbdyn
{

/** This structure defines a body sensor, that is a sensor that provides
 * dynamic information about a body. It would typically be used to represent an
 * IMU reading but in more ideal (simulation, external tracking system...) it
 * can hold more information */
struct BodySensor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** Default constructor, does not represent a valid body sensor */
  BodySensor()
  : BodySensor("", "", Eigen::Vector3d::Zero())
  {
  }

  /** Constructor
   *
   * @param name Name of the sensor
   *
   * @param bodyName Name of the body to which the sensor is attached
   *
   */
  BodySensor(const std::string & name,
             const std::string & bodyName,
             const Eigen::Vector3d & position)
  : name_(name),
    bodyName_(bodyName),
    position_(position)
  {
  }

  /** Get the sensor's name */
  inline const std::string & name() const
  {
    return name_;
  }

  /** Get the sensor's parent body name */
  inline const std::string & parentBody() const
  {
    return bodyName_;
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

  /** Set the sensor's orientation reading */
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

  /** Return the sensor's acceleration reading, Zero if not provided */
  inline const Eigen::Vector3d & acceleration() const
  {
    return acceleration_;
  }

  /** Set the sensor's acceleration reading */
  inline void acceleration(const Eigen::Vector3d & acceleration)
  {
    acceleration_ = acceleration;
  }
private:
  std::string name_;
  std::string bodyName_;
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();
};

typedef std::vector<BodySensor, Eigen::aligned_allocator<BodySensor>> BodySensorVector;

}
