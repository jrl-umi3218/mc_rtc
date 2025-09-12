/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/contact_transform.h>
#include <mc_rbdyn/surface_utils.h>

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
  for(sva::PTransformd & p : impl->pointsFromOrigin) { points().push_back(p * X_b_s()); }
}

void GripperSurface::originTransform(const sva::PTransformd & X_s_sp)
{
  for(sva::PTransformd & p : impl->pointsFromOrigin) { p = p * X_s_sp.inv(); }
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

std::unique_ptr<GripperSurface> GripperSurface::fromXML(const tinyxml2::XMLElement & elem)
{
  std::string name = elem.Attribute("name");
  std::string bodyName = elem.Attribute("link");
  sva::PTransformd X_b_s = tfFromOriginDom(*elem.FirstChildElement("origin"));
  std::string materialName;
  if(auto * matElem = elem.FirstChildElement("material")) materialName = matElem->Attribute("name");

  // Motor
  auto * motorElem = elem.FirstChildElement("motor");
  double motorMaxTorque = 0.0;
  sva::PTransformd X_b_motor;
  if(motorElem)
  {
    motorMaxTorque = motorElem->DoubleAttribute("max_torque");
    X_b_motor = tfFromOriginDom(*motorElem);
  }

  // Points
  std::vector<sva::PTransformd> points;
  auto * pointsElem = elem.FirstChildElement("points");
  if(pointsElem)
  {
    for(auto * pointElem = pointsElem->FirstChildElement("origin"); pointElem;
        pointElem = pointElem->NextSiblingElement("origin"))
    {
      points.push_back(tfFromOriginDom(*pointElem));
    }
  }

  return std::make_unique<GripperSurface>(name, bodyName, X_b_s, materialName, points, X_b_motor, motorMaxTorque);
}

tinyxml2::XMLElement * GripperSurface::toXML(tinyxml2::XMLDocument & doc) const
{
  auto * gripElem = doc.NewElement("gripper_surface");
  gripElem->SetAttribute("name", name().c_str());
  gripElem->SetAttribute("link", bodyName().c_str());

  // Origin
  gripElem->InsertEndChild(tfToOriginDom(doc, X_b_s(), "origin"));

  // Material
  if(!materialName().empty())
  {
    auto * matElem = doc.NewElement("material");
    matElem->SetAttribute("name", materialName().c_str());
    gripElem->InsertEndChild(matElem);
  }

  // Motor
  auto * motorElem = doc.NewElement("motor");
  motorElem->SetAttribute("max_torque", fmt::format("{:.8f}", impl->motorMaxTorque).c_str());
  motorElem->InsertEndChild(tfToOriginDom(doc, impl->X_b_motor, "origin"));
  gripElem->InsertEndChild(motorElem);

  // Points
  auto * pointsElem = doc.NewElement("points");
  for(const auto & p : impl->pointsFromOrigin) { pointsElem->InsertEndChild(tfToOriginDom(doc, p, "origin")); }
  gripElem->InsertEndChild(pointsElem);

  return gripElem;
}

} // namespace mc_rbdyn
