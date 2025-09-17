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
