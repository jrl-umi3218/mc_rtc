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

namespace details
{
/** Polyhedron should display a polyhedron
 *
 * \tparam GetTrianglesT Should return a vector of triangles with points ordered clockwise, expressed as either an
 * std::vector<std::array<double, 3>>
 */
template<typename GetTrianglesT>
struct PolyhedronImpl : public Element
{
  static constexpr auto type = Elements::Polyhedron;

  PolyhedronImpl(const std::string & name, const PolyhedronConfig & config, GetTrianglesT get_triangles_fn)
  : Element(name), config_(config), get_triangles_fn_(get_triangles_fn)
  {
    static_assert(details::CheckReturnType<GetTrianglesT, std::vector<std::array<Eigen::Vector3d, 3>>>::value,
                  "Polyhedron callback must return a vector of triangles with points ordered clockwise, expressed as "
                  "an std::vector<std::array<double, 3>>");
  }

  /** Invalid element */
  PolyhedronImpl() {}

  static constexpr size_t write_size()
  {
    return Element::write_size() + 2 + PolyhedronConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(get_triangles_fn_());
    builder.write();
    config_.write(builder);
  }

protected:
  PolyhedronConfig config_;
  GetTrianglesT get_triangles_fn_;
};

/** ColoredPolyhedron should display a polyhedron
 *
 * \tparam GetTrianglesT \see PolyhedronImpl
 * \tparam GetColorT Should return a vector of triangle colors expressed as an
 * std::vector<std::array<mc_rtc::gui::Color, 3>>
 */
template<typename GetTrianglesT, typename GetColorT>
struct ColoredPolyhedronImpl : public PolyhedronImpl<GetTrianglesT>
{
  ColoredPolyhedronImpl(const std::string & name,
                        const PolyhedronConfig & config,
                        GetTrianglesT get_triangles_fn,
                        GetColorT get_color_fn)
  : PolyhedronImpl<GetTrianglesT>(name, config, get_triangles_fn), get_color_fn_(get_color_fn)
  {
    static_assert(details::CheckReturnType<GetColorT, std::vector<std::array<mc_rtc::gui::Color, 3>>>::value,
                  "Polyhedron color callback must return an std::vector<std::array<mc_rtc::gui::Color, 3>> (color for "
                  "each of the triangle vertices)");
  }

  /** Invalid element */
  ColoredPolyhedronImpl() {}

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(get_triangles_fn_());
    const auto & colors = get_color_fn_();
    builder.start_array(colors.size());
    for(const auto & c : colors)
    {
      builder.start_array(3);
      c[0].write(builder);
      c[1].write(builder);
      c[2].write(builder);
      builder.finish_array();
    }
    builder.finish_array();
    config_.write(builder);
  }

protected:
  using PolyhedronImpl<GetTrianglesT>::config_;
  using PolyhedronImpl<GetTrianglesT>::get_triangles_fn_;
  GetColorT get_color_fn_;
};

} // namespace details

/** Helper function to build a PolyhedronImpl */
template<typename GetVerticesT, typename GetColorT>
details::ColoredPolyhedronImpl<GetVerticesT, GetColorT> Polyhedron(const std::string & name,
                                                                   GetVerticesT get_triangles_fn,
                                                                   GetColorT get_color_fn)
{
  return details::ColoredPolyhedronImpl<GetVerticesT, GetColorT>(name, {}, get_triangles_fn, get_color_fn);
}

/** Helper function to build a PolyhedronImpl */
template<typename GetVerticesT, typename GetColorT>
details::ColoredPolyhedronImpl<GetVerticesT, GetColorT> Polyhedron(const std::string & name,
                                                                   const PolyhedronConfig & config,
                                                                   GetVerticesT get_triangles_fn,
                                                                   GetColorT get_color_fn)
{
  return details::ColoredPolyhedronImpl<GetVerticesT, GetColorT>(name, config, get_triangles_fn, get_color_fn);
}

/** Helper function to build a PolyhedronImpl */
template<typename GetVerticesT>
details::PolyhedronImpl<GetVerticesT> Polyhedron(const std::string & name, GetVerticesT get_triangles_fn)
{
  return details::PolyhedronImpl<GetVerticesT>(name, {}, get_triangles_fn);
}

/** Helper function to build a PolyhedronImpl */
template<typename GetVerticesT>
details::PolyhedronImpl<GetVerticesT> Polyhedron(const std::string & name,
                                                 const PolyhedronConfig & config,
                                                 GetVerticesT get_triangles_fn)
{
  return details::PolyhedronImpl<GetVerticesT>(name, config, get_triangles_fn);
}

} // namespace gui

} // namespace mc_rtc
