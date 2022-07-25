/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/gui/types.h>

namespace mc_rtc
{

namespace gui
{

/** Polyhedron should display a polyhedron or a set of polyhedrons
 *
 * A color is provided to display the polyhedron(s)
 *
 * \tparam GetT Should return an std::vector<Eigen::Vector3d> (one polyhedron) or an
 * std::vector<std::vector<Eigen:Vector3d>> (list of polyhedrons)
 *
 */
template<typename GetVerticesT, typename GetColorT>
struct PolyhedronImpl : public Element
{
  static constexpr auto type = Elements::Polyhedron;

  PolyhedronImpl(const std::string & name,
                 const PolyhedronConfig & config,
                 GetVerticesT get_vertices_fn,
                 GetColorT get_color_fn)
  : Element(name), config_(config), get_vertices_fn_(get_vertices_fn), get_color_fn_(get_color_fn)
  {
    static_assert(details::CheckReturnType<GetVerticesT, PolyhedronTriangles>::value,
                  "Polyhedron vertices callback must return an std::vector<Eigen::Vector3d> (triangle set with points "
                  "ordered clockwise)");
    static_assert(details::CheckReturnType<GetColorT, PolyhedronColors>::value,
                  "Polyhedron vertices' color callback must return an std::vector<Eigen::Vector4d> (triangle set with "
                  "points ordered clockwise)");
  }

  /** Invalid element */
  PolyhedronImpl() {}

  static constexpr size_t write_size()
  {
    return Element::write_size() + 2 + PolyhedronConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(get_vertices_fn_());
    builder.write(get_color_fn_());
    config_.write(builder);
  }

private:
  PolyhedronConfig config_;
  GetVerticesT get_vertices_fn_;
  GetColorT get_color_fn_;
};

/** Helper function to build a PolyhedronImpl */
template<typename GetVerticesT, typename GetColorT>
PolyhedronImpl<GetVerticesT, GetColorT> Polyhedron(const std::string & name,
                                                   GetVerticesT get_vertices_fn,
                                                   GetColorT get_color_fn)
{
  return PolyhedronImpl<GetVerticesT, GetColorT>(name, {}, get_vertices_fn, get_color_fn);
}

/** Helper function to build a PolyhedronImpl */
template<typename GetVerticesT, typename GetColorT>
PolyhedronImpl<GetVerticesT, GetColorT> Polyhedron(const std::string & name,
                                                   const PolyhedronConfig & config,
                                                   GetVerticesT get_vertices_fn,
                                                   GetColorT get_color_fn)
{
  return PolyhedronImpl<GetVerticesT, GetColorT>(name, config, get_vertices_fn, get_color_fn);
}

} // namespace gui

} // namespace mc_rtc
