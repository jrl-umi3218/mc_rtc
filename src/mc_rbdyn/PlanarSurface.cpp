/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/contact_transform.h>
#include <mc_rbdyn/surface_utils.h>
#include <fmt/format.h>
#include <tinyxml2.h>

namespace mc_rbdyn
{

struct PlanarSurfaceImpl
{
public:
  std::vector<std::pair<double, double>> planarPoints;
};

PlanarSurface::PlanarSurface(const std::string & name,
                             const std::string & bodyName,
                             const sva::PTransformd & X_b_s,
                             const std::string & materialName,
                             const std::vector<std::pair<double, double>> & planarPoints)
: Surface(name, bodyName, X_b_s, materialName), impl(new PlanarSurfaceImpl({planarPoints}))
{
  computePoints();
}

PlanarSurface::~PlanarSurface() {}

void PlanarSurface::computePoints()
{
  points().clear();
  for(std::pair<double, double> & p : impl->planarPoints)
  {
    points().push_back(sva::PTransformd(Eigen::Vector3d(p.first, p.second, 0)) * X_b_s());
  }
}

void PlanarSurface::planarTransform(const double & T, const double & B, const double & N_rot)
{
  sva::PTransformd X = mc_rbdyn::planar(T, B, N_rot);
  sva::PTransformd X_b_s_new = X * X_b_s();
  std::vector<std::pair<double, double>> newPlanarPoints(0);
  for(std::pair<double, double> & p : impl->planarPoints)
  {
    sva::PTransformd newP = sva::PTransformd(Eigen::Vector3d(p.first, p.second, 0)) * (X.inv());
    newPlanarPoints.push_back(std::pair<double, double>(newP.translation().x(), newP.translation().y()));
  }
  impl->planarPoints = newPlanarPoints;
  // Call compute points
  X_b_s(X_b_s_new);
}

const std::vector<std::pair<double, double>> & PlanarSurface::planarPoints() const
{
  return impl->planarPoints;
}

void PlanarSurface::planarPoints(const std::vector<std::pair<double, double>> & planarPoints)
{
  impl->planarPoints = planarPoints;
  computePoints();
}

std::shared_ptr<Surface> PlanarSurface::copy() const
{
  return std::shared_ptr<Surface>(new PlanarSurface(name(), bodyName(), X_b_s(), materialName(), impl->planarPoints));
}

std::string PlanarSurface::type() const
{
  return "planar";
}

tinyxml2::XMLElement * PlanarSurface::toXML(tinyxml2::XMLDocument & doc) const
{
  auto * planarElem = doc.NewElement("planar_surface");
  planarElem->SetAttribute("name", name().c_str());
  planarElem->SetAttribute("link", bodyName().c_str());

  // Origin
  planarElem->InsertEndChild(tfToOriginDom(doc, X_b_s(), "origin"));

  // Points
  auto * pointsElem = doc.NewElement("points");
  for(const auto & pt : planarPoints())
  {
    auto * pointElem = doc.NewElement("point");
    pointElem->SetAttribute("xy", fmt::format("{:.8f} {:.8f}", pt.first, pt.second).c_str());
    pointsElem->InsertEndChild(pointElem);
  }
  planarElem->InsertEndChild(pointsElem);

  // Material
  if(!materialName().empty())
  {
    auto * matElem = doc.NewElement("material");
    matElem->SetAttribute("name", materialName().c_str());
    planarElem->InsertEndChild(matElem);
  }

  return planarElem;
}

} // namespace mc_rbdyn
