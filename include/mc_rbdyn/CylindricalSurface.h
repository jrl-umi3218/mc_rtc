/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

struct Robot;

struct CylindricalSurfaceImpl;

struct MC_RBDYN_DLLAPI CylindricalSurface : public Surface
{
  CylindricalSurface(const std::string & name,
                     const std::string & bodyName,
                     const sva::PTransformd & X_b_s,
                     const std::string & materialName,
                     const double & radius,
                     const double & width);

  ~CylindricalSurface();

  void computePoints() override;

  const double & radius() const;

  const double & width() const;

  void width(const double & width);

  std::shared_ptr<Surface> copy() const override;

  std::string type() const override;

private:
  std::unique_ptr<CylindricalSurfaceImpl> impl;
};

} // namespace mc_rbdyn
