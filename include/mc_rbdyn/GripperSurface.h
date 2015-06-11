#pragma once

#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

struct GripperSurfaceImpl;

struct GripperSurface : public Surface
{
public:
  GripperSurface(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s, const std::string & materialName, const std::vector<sva::PTransformd> & pointsFromOrigin, const sva::PTransformd & X_b_motor, const double & motorMaxTorque);

  ~GripperSurface();

  virtual void computePoints();

  void originTransform(const sva::PTransformd & X_s_sp);

  virtual std::shared_ptr<Surface> copy() const;

  virtual std::string type() const override;

  const std::vector<sva::PTransformd> & pointsFromOrigin() const;

  const sva::PTransformd & X_b_motor() const;

  const double & motorMaxTorque() const;
private:
  std::unique_ptr<GripperSurfaceImpl> impl;
};

}
