/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

struct GripperSurfaceImpl;

struct MC_RBDYN_DLLAPI GripperSurface : public Surface
{
public:
  GripperSurface(const std::string & name,
                 const std::string & bodyName,
                 const sva::PTransformd & X_b_s,
                 const std::string & materialName,
                 const std::vector<sva::PTransformd> & pointsFromOrigin,
                 const sva::PTransformd & X_b_motor,
                 const double & motorMaxTorque);

  ~GripperSurface() override;

  /**
   * @brief Construct a GripperSurface from an XML element.
   *
   * Example XML:
   * @code{.xml}
   *  <gripper_surface name="LeftGripper" link="l_wrist">
   *    <origin rpy="3.14 0.0 0.0" xyz="0.0 -0.0085 -0.095" />
   *    <motor rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0" max_torque="1000" />
   *    <points>
   *      <origin rpy="-1.5707964897155762 -0.0 0.0" xyz="0.0 0.02 0.0" />
   *      <origin rpy="1.5707969665527344 -0.0 0.0" xyz="0.0 -0.02 0.0" />
   *      <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.015" />
   *      <origin rpy="3.1415939331054688 -0.0 0.0" xyz="0.0 0.0 -0.005" />
   *    </points>
   *    <material name="plastic" />
   *  </gripper_surface>
   * @endcode
   *
   * @param elem The XML element describing the gripper surface.
   * @return Unique pointer to the constructed GripperSurface.
   */
  static std::unique_ptr<GripperSurface> fromXML(const tinyxml2::XMLElement & elem);

  void computePoints() override;

  void originTransform(const sva::PTransformd & X_s_sp);

  std::shared_ptr<Surface> copy() const override;

  std::string type() const override;

  const std::vector<sva::PTransformd> & pointsFromOrigin() const;

  const sva::PTransformd & X_b_motor() const;

  const double & motorMaxTorque() const;

  tinyxml2::XMLElement * toXML(tinyxml2::XMLDocument & doc) const override;

private:
  std::unique_ptr<GripperSurfaceImpl> impl;
};

} // namespace mc_rbdyn
