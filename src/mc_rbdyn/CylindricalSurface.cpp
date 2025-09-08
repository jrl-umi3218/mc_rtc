/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/contact_transform.h>
#include <mc_rbdyn/surface_utils.h>

namespace mc_rbdyn
{

struct CylindricalSurfaceImpl
{
public:
  double radius;
  double width;
};

CylindricalSurface::CylindricalSurface(const std::string & name,
                                       const std::string & bodyName,
                                       const sva::PTransformd & X_b_s,
                                       const std::string & materialName,
                                       const double & radius,
                                       const double & width)
: Surface(name, bodyName, X_b_s, materialName), impl(new CylindricalSurfaceImpl({radius, width}))
{
  computePoints();
}

CylindricalSurface::~CylindricalSurface() {}

void CylindricalSurface::computePoints()
{
  points().clear();
  points().push_back(sva::PTransformd(Eigen::Vector3d(-impl->width / 2, 0, 0)) * X_b_s());
  points().push_back(sva::PTransformd(Eigen::Vector3d(impl->width / 2, 0, 0)) * X_b_s());
}

const double & CylindricalSurface::radius() const
{
  return impl->radius;
}

const double & CylindricalSurface::width() const
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
  return std::shared_ptr<Surface>(
      new CylindricalSurface(name(), bodyName(), X_b_s(), materialName(), impl->radius, impl->width));
}

std::string CylindricalSurface::type() const
{
  return "cylindrical";
}

std::unique_ptr<CylindricalSurface> CylindricalSurface::fromXML(const tinyxml2::XMLElement & elem)
{
  std::string name = elem.Attribute("name");
  std::string bodyName = elem.Attribute("link");
  sva::PTransformd X_b_s = tfFromOriginDom(*elem.FirstChildElement("origin"));
  double radius = elem.DoubleAttribute("radius");
  double width = elem.DoubleAttribute("width");
  std::string materialName;
  if(auto * matElem = elem.FirstChildElement("material")) materialName = matElem->Attribute("name");
  return std::make_unique<CylindricalSurface>(name, bodyName, X_b_s, materialName, radius, width);
}
tinyxml2::XMLElement * CylindricalSurface::toXML(tinyxml2::XMLDocument & doc) const
{
  auto * cylElem = doc.NewElement("cylindrical_surface");
  cylElem->SetAttribute("name", name().c_str());
  cylElem->SetAttribute("link", bodyName().c_str());

  // Origin
  cylElem->InsertEndChild(tfToOriginDom(doc, X_b_s(), "origin"));

  // Radius and width
  cylElem->SetAttribute("radius", fmt::format("{:.8f}", radius()).c_str());
  cylElem->SetAttribute("width", fmt::format("{:.8f}", width()).c_str());

  // Points
  auto * pointsElem = doc.NewElement("points");
  for(const auto & pt : points())
  {
    auto * pointElem = doc.NewElement("point");
    pointElem->SetAttribute(
        "xyz",
        fmt::format("{:.8f} {:.8f} {:.8f}", pt.translation().x(), pt.translation().y(), pt.translation().z()).c_str());
    pointsElem->InsertEndChild(pointElem);
  }
  cylElem->InsertEndChild(pointsElem);

  // Material
  if(!materialName().empty())
  {
    auto * matElem = doc.NewElement("material");
    matElem->SetAttribute("name", materialName().c_str());
    cylElem->InsertEndChild(matElem);
  }

  return cylElem;
}

} // namespace mc_rbdyn
