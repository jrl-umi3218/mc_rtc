#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/contact_transform.h>

namespace mc_rbdyn
{

struct GripperSurfaceImpl
{
public:
  std::vector<sva::PTransformd> pointsFromOrigin;
  sva::PTransformd X_b_motor;
  double motorMaxTorque;
};

GripperSurface::GripperSurface(const std::string & name,
                               const std::string & bodyName,
                               const sva::PTransformd & X_b_s,
                               const std::string & materialName,
                               const std::vector<sva::PTransformd> & pointsFromOrigin,
                               const sva::PTransformd & X_b_motor,
                               const double & motorMaxTorque)
: Surface(name, bodyName, X_b_s, materialName),
  impl(new GripperSurfaceImpl({pointsFromOrigin, X_b_motor, motorMaxTorque}))
{
  computePoints();
}

GripperSurface::~GripperSurface() {}

void GripperSurface::computePoints()
{
  points().clear();
  for(sva::PTransformd & p : impl->pointsFromOrigin)
  {
    points().push_back(p * X_b_s());
  }
}

void GripperSurface::originTransform(const sva::PTransformd & X_s_sp)
{
  for(sva::PTransformd & p : impl->pointsFromOrigin)
  {
    p = p * X_s_sp.inv();
  }
  X_b_s(X_s_sp * X_b_s());
}

std::shared_ptr<Surface> GripperSurface::copy() const
{
  return std::shared_ptr<Surface>(new GripperSurface(name(), bodyName(), X_b_s(), materialName(),
                                                     impl->pointsFromOrigin, impl->X_b_motor, impl->motorMaxTorque));
}

std::string GripperSurface::type() const
{
  return "gripper";
}

const std::vector<sva::PTransformd> & GripperSurface::pointsFromOrigin() const
{
  return impl->pointsFromOrigin;
}

const sva::PTransformd & GripperSurface::X_b_motor() const
{
  return impl->X_b_motor;
}

const double & GripperSurface::motorMaxTorque() const
{
  return impl->motorMaxTorque;
}

} // namespace mc_rbdyn
