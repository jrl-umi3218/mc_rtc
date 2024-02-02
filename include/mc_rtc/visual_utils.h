/*
 * Copyright 2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/types.h>

#include <RBDyn/parsers/common.h>

/** This header provides a number of function to help with the creation of rbd::parsers::Visual elements */

namespace mc_rtc
{

namespace details
{

/** Set the color of a visual element */
inline void setVisualColor(rbd::parsers::Visual & visual, const mc_rtc::gui::Color & color)
{
  rbd::parsers::Material mat;
  rbd::parsers::Material::Color col;
  col.r = color.r;
  col.g = color.g;
  col.b = color.b;
  col.a = color.a;
  mat.type = rbd::parsers::Material::Type::COLOR;
  mat.data = col;
  visual.material = mat;
}

/** Initialize a new Visual object */
template<typename DataT>
rbd::parsers::Visual makeVisual(const DataT & data, const mc_rtc::gui::Color & color)
{
  using GeometryT = rbd::parsers::Geometry;
  rbd::parsers::Visual out;
  out.origin = sva::PTransformd::Identity();
  out.geometry.type = []()
  {
    if constexpr(std::is_same_v<DataT, GeometryT::Box>) { return GeometryT::Type::BOX; }
    else if constexpr(std::is_same_v<DataT, GeometryT::Cylinder>) { return GeometryT::Type::CYLINDER; }
    else if constexpr(std::is_same_v<DataT, GeometryT::Sphere>) { return GeometryT::Type::SPHERE; }
    else if constexpr(std::is_same_v<DataT, GeometryT::Mesh>) { return GeometryT::Type::MESH; }
    else if constexpr(std::is_same_v<DataT, GeometryT::Superellipsoid>) { return GeometryT::Type::SUPERELLIPSOID; }
    else { static_assert(!std::is_same_v<DataT, DataT>); }
  }();
  out.geometry.data = data;
  setVisualColor(out, color);
  return out;
}

/** Returns the geometry data inside a Visual object */
template<rbd::parsers::Geometry::Type type>
auto & getVisualGeometry(rbd::parsers::Visual & visual)
{
  if(visual.geometry.type != type) { mc_rtc::log::error_and_throw("Visual type not matching the expected type"); }
  using Type = rbd::parsers::Geometry::Type;
  auto & data = visual.geometry.data;
  if constexpr(type == Type::BOX) { return boost::get<rbd::parsers::Geometry::Box>(data); }
  else if constexpr(type == Type::CYLINDER) { return boost::get<rbd::parsers::Geometry::Cylinder>(data); }
  else if constexpr(type == Type::SPHERE) { return boost::get<rbd::parsers::Geometry::Sphere>(data); }
  else if constexpr(type == Type::MESH) { return boost::get<rbd::parsers::Geometry::Mesh>(data); }
  else if constexpr(type == Type::SUPERELLIPSOID) { return boost::get<rbd::parsers::Geometry::Superellipsoid>(data); }
  else { static_assert(static_cast<int>(type) != static_cast<int>(type)); }
}

} // namespace details

/** Returns a new Visual that holds a Sphere object */
inline rbd::parsers::Visual makeVisualSphere(double radius, const mc_rtc::gui::Color & color)
{
  rbd::parsers::Geometry::Sphere s;
  s.radius = radius;
  return details::makeVisual(s, color);
}

/** Get the sphere data if the visual is a sphere, throws otherwise */
inline constexpr auto getVisualSphere = details::getVisualGeometry<rbd::parsers::Geometry::Type::SPHERE>;

/** Returns a new Visual that holds a Cylinder object */
inline rbd::parsers::Visual makeVisualCylinder(double radius, double length, const mc_rtc::gui::Color & color)
{
  rbd::parsers::Geometry::Cylinder c;
  c.length = length;
  c.radius = radius;
  return details::makeVisual(c, color);
}

/** Get the cylinder data if the visual is a cylinder, throws otherwise */
inline constexpr auto getVisualCylinder = details::getVisualGeometry<rbd::parsers::Geometry::Type::CYLINDER>;

/** Returns a new Visual that holds a Box object */
inline rbd::parsers::Visual makeVisualBox(const Eigen::Vector3d & dim, const mc_rtc::gui::Color & color)
{
  rbd::parsers::Geometry::Box b;
  b.size = dim;
  return details::makeVisual(b, color);
}

/** Get the box data if the visual is a box, throws otherwise */
inline constexpr auto getVisualBox = details::getVisualGeometry<rbd::parsers::Geometry::Type::BOX>;

/** Returns a new Visual that holds a Mesh object */
inline rbd::parsers::Visual makeVisualMesh(const std::string & path, Eigen::Vector3d scaleV)
{
  rbd::parsers::Geometry::Mesh m;
  m.filename = path;
  m.scaleV = scaleV;
  return details::makeVisual(m, {});
}

/** Returns a new Visual that holds a Mesh object */
inline rbd::parsers::Visual makeVisualMesh(const std::string & path, double scale)
{
  return makeVisualMesh(path, Eigen::Vector3d(scale, scale, scale));
}

/** Get the mesh data if the visual is a mesh, throws otherwise */
inline constexpr auto getVisualMesh = details::getVisualGeometry<rbd::parsers::Geometry::Type::MESH>;

/** Returns a new Visual that holds a Superellipsoid object */
inline rbd::parsers::Visual makeVisualSuperellispoid(const Eigen::Vector3d & size,
                                                     double epsilon1,
                                                     double epsilon2,
                                                     const mc_rtc::gui::Color & color)
{
  rbd::parsers::Geometry::Superellipsoid se;
  se.size = size;
  se.epsilon1 = epsilon1;
  se.epsilon2 = epsilon2;
  return details::makeVisual(se, color);
}

/** Get the superellipsoid data if the visual is a superellipsoid, throws otherwise */
inline constexpr auto getVisualSuperellipsoid =
    details::getVisualGeometry<rbd::parsers::Geometry::Type::SUPERELLIPSOID>;

} // namespace mc_rtc
