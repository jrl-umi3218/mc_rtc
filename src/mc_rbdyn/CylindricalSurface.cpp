#include <mc_rbdyn/CylindricalSurface.h>

#include <mc_rbdyn/contact_transform.h>
#include <mc_rbdyn/Robots.h>

namespace mc_rbdyn
{

struct CylindricalSurfaceImpl
{
public:
  double radius;
  double width;
};

CylindricalSurface::CylindricalSurface(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s, const std::string & materialName, const double & radius, const double & width)
: Surface(name, bodyName, X_b_s, materialName),
  impl(new CylindricalSurfaceImpl({radius, width}))
{
  computePoints();
}

CylindricalSurface::~CylindricalSurface()
{
}

void CylindricalSurface::computePoints()
{
  points().clear();
  points().push_back(sva::PTransformd(Eigen::Vector3d(-impl->width/2,0,0))*X_b_s());
  points().push_back(sva::PTransformd(Eigen::Vector3d(impl->width/2,0,0))*X_b_s());
}

const double & CylindricalSurface::radius() const
{
  return impl->radius;
}

const double& CylindricalSurface::width() const
{
  return impl->width;
}

void CylindricalSurface::width(const double & width)
{
  impl->width = width;
  computePoints();
}

std::shared_ptr<Surface> CylindricalSurface::copy() const
{
  return std::shared_ptr<Surface>(new CylindricalSurface(name(), bodyName(), X_b_s(), materialName(), impl->radius, impl->width));
}

std::string CylindricalSurface::type() const
{
  return "cylindrical";
}

}
