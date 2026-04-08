/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/gui/types.h>

namespace mc_rtc::gui
{

namespace details
{

/** Polyhedron should display a polyhedron
 *
 * \tparam GetTrianglesT Should return a vector of triangles with points ordered clockwise, expressed as a
 * std::vector<std::array<double, 3>>
 */
template<typename GetTrianglesT>
struct PolyhedronTrianglesListImpl : public Element
{
  static constexpr auto type = Elements::PolyhedronTrianglesList;

  PolyhedronTrianglesListImpl(const std::string & name, const PolyhedronConfig & config, GetTrianglesT get_triangles_fn)
  : Element(name), config_(config), get_triangles_fn_(get_triangles_fn)
  {
    static_assert(details::CheckReturnType<GetTrianglesT, std::vector<std::array<Eigen::Vector3d, 3>>>::value,
                  "Polyhedron callback must return a vector of triangles with points ordered clockwise, expressed as "
                  "an std::vector<std::array<double, 3>>");
  }

  /** Invalid element */
  PolyhedronTrianglesListImpl() {}

  static constexpr size_t write_size() { return Element::write_size() + 1 + PolyhedronConfig::write_size(); }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(get_triangles_fn_());
    config_.write(builder);
  }

protected:
  PolyhedronConfig config_;
  GetTrianglesT get_triangles_fn_;
};

/** Polyhedron should display a polyhedron
 *
 * \tparam GetVerticesT Should return a list of vertices as an std::vector<Eigen::Vector3d>
 *
 * \tparam GetTrianglesT Should return a list of triangle as std::vector<std::array<size_t, 3>>
 */
template<typename GetVerticesT, typename GetTrianglesT>
struct PolyhedronVerticesTrianglesImpl : public Element
{
  static constexpr auto type = Elements::PolyhedronVerticesTriangles;

  PolyhedronVerticesTrianglesImpl(const std::string & name,
                                  const PolyhedronConfig & config,
                                  GetVerticesT get_vertices_fn,
                                  GetTrianglesT get_triangles_fn)
  : Element(name), config_(config), get_vertices_fn_(get_vertices_fn), get_triangles_fn_(get_triangles_fn)
  {
    static_assert(details::CheckReturnType<GetVerticesT, std::vector<Eigen::Vector3d>>::value,
                  "Polyhedron vertices callback must return a vector of 3D points");
    static_assert(details::CheckReturnType<GetTrianglesT, std::vector<std::array<size_t, 3>>>::value,
                  "Polyhedron triangles callback must return a list of 3-uple index into vertices array");
  }

  /** Invalid element */
  PolyhedronVerticesTrianglesImpl() {}

  static constexpr size_t write_size() { return Element::write_size() + 2 + PolyhedronConfig::write_size(); }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(get_vertices_fn_());
    builder.write(get_triangles_fn_());
    config_.write(builder);
  }

protected:
  PolyhedronConfig config_;
  GetVerticesT get_vertices_fn_;
  GetTrianglesT get_triangles_fn_;
};

/** ColoredPolyhedron should display a polyhedron using the color information provided
 *
 * \tparam PolyhedronT this represents
 * \tparam GetColorT Should return a vector of triangle colors expressed as an
 * std::vector<std::array<mc_rtc::gui::Color, 3>>
 */
template<typename PolyhedronT, typename GetColorT>
struct ColoredPolyhedronImpl : public PolyhedronT
{
  ColoredPolyhedronImpl(PolyhedronT && poly, GetColorT get_color_fn) : PolyhedronT(poly), get_color_fn_(get_color_fn)
  {
    if constexpr(PolyhedronT::type == Elements::PolyhedronTrianglesList)
    {
      static_assert(
          details::CheckReturnType<GetColorT, std::vector<std::array<mc_rtc::gui::Color, 3>>>::value,
          "Polyhedron color callback must return an std::vector<std::array<mc_rtc::gui::Color, 3>> (color for "
          "each of the triangle vertices)");
    }
    else
    {
      static_assert(details::CheckReturnType<GetColorT, std::vector<mc_rtc::gui::Color>>::value,
                    "Polyhedron color callback must return an std::vector<mc_rtc::gui::Color> (color for "
                    "each of the vertices)");
    }
  }

  /** Invalid element */
  ColoredPolyhedronImpl() {}

  static constexpr size_t write_size() { return PolyhedronT::write_size() + 1; }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    PolyhedronT::write(builder);
    const auto & colors = get_color_fn_();
    builder.start_array(colors.size());
    for(const auto & c : colors)
    {
      if constexpr(PolyhedronT::type == Elements::PolyhedronTrianglesList)
      {
        builder.start_array(3);
        c[0].write(builder);
        c[1].write(builder);
        c[2].write(builder);
        builder.finish_array();
      }
      else
      {
        c.write(builder);
      }
    }
    builder.finish_array();
  }

protected:
  GetColorT get_color_fn_;
};

} // namespace details

/** Helper function to build a PolyhedronTriangleListImpl */
template<typename GetTrianglesT>
auto Polyhedron(const std::string & name, GetTrianglesT get_triangles_fn)
{
  return details::PolyhedronTrianglesListImpl<GetTrianglesT>(name, {}, get_triangles_fn);
}

/** Helper function to build a PolyhedronTriangleListImpl */
template<typename GetTrianglesT>
auto Polyhedron(const std::string & name, const PolyhedronConfig & config, GetTrianglesT get_triangles_fn)
{
  return details::PolyhedronTrianglesListImpl<GetTrianglesT>(name, config, get_triangles_fn);
}

/** Helper function to build a PolyhedronVerticesTrianglesImpl */
template<typename GetVerticesOrTrianglesT, typename GetTrianglesOrColorsT>
auto Polyhedron(const std::string & name,
                GetVerticesOrTrianglesT get_vertices_or_triangles_fn,
                GetTrianglesOrColorsT get_triangles_or_colors_fn)
{
  if constexpr(details::CheckReturnType<GetVerticesOrTrianglesT, std::vector<std::array<Eigen::Vector3d, 3>>>::value)
  {
    auto poly = Polyhedron(name, PolyhedronConfig{}, get_vertices_or_triangles_fn);
    return details::ColoredPolyhedronImpl<decltype(poly), GetTrianglesOrColorsT>(std::move(poly),
                                                                                 get_triangles_or_colors_fn);
  }
  else
  {
    return details::PolyhedronVerticesTrianglesImpl<GetVerticesOrTrianglesT, GetTrianglesOrColorsT>(
        name, {}, get_vertices_or_triangles_fn, get_triangles_or_colors_fn);
  }
}

/** Helper function to build a PolyhedronVerticesTrianglesImpl */
template<typename GetVerticesOrTrianglesT, typename GetTrianglesOrColorsT>
auto Polyhedron(const std::string & name,
                const PolyhedronConfig & config,
                GetVerticesOrTrianglesT get_vertices_or_triangles_fn,
                GetTrianglesOrColorsT get_triangles_or_colors_fn)
{
  if constexpr(details::CheckReturnType<GetVerticesOrTrianglesT, std::vector<std::array<Eigen::Vector3d, 3>>>::value)
  {
    auto poly = Polyhedron(name, config, get_vertices_or_triangles_fn);
    return details::ColoredPolyhedronImpl<decltype(poly), GetTrianglesOrColorsT>(std::move(poly),
                                                                                 get_triangles_or_colors_fn);
  }
  else
  {
    return details::PolyhedronVerticesTrianglesImpl<GetVerticesOrTrianglesT, GetTrianglesOrColorsT>(
        name, config, get_vertices_or_triangles_fn, get_triangles_or_colors_fn);
  }
}

/** Helper function to build a ColoredPolyhedronImpl */
template<typename GetTrianglesT, typename GetColorT>
auto ColoredPolyhedron(const std::string & name,
                       const PolyhedronConfig & config,
                       GetTrianglesT get_triangles_fn,
                       GetColorT get_color_fn)
{
  auto poly = Polyhedron(name, config, get_triangles_fn);
  return details::ColoredPolyhedronImpl<decltype(poly), GetColorT>(std::move(poly), get_color_fn);
}

/** Helper function to build a ColoredPolyhedronImpl */
template<typename GetVerticesT, typename GetTrianglesT, typename GetColorT>
auto Polyhedron(const std::string & name,
                GetVerticesT get_vertices_fn,
                GetTrianglesT get_triangles_fn,
                GetColorT get_color_fn)
{
  auto poly = Polyhedron(name, get_vertices_fn, get_triangles_fn);
  return details::ColoredPolyhedronImpl<decltype(poly), GetColorT>(std::move(poly), get_color_fn);
}

/** Helper function to build a ColoredPolyhedronImpl */
template<typename GetVerticesT, typename GetTrianglesT, typename GetColorT>
auto Polyhedron(const std::string & name,
                const PolyhedronConfig & config,
                GetVerticesT get_vertices_fn,
                GetTrianglesT get_triangles_fn,
                GetColorT get_color_fn)
{
  auto poly = Polyhedron(name, config, get_vertices_fn, get_triangles_fn);
  return details::ColoredPolyhedronImpl<decltype(poly), GetColorT>(std::move(poly), get_color_fn);
}

} // namespace mc_rtc::gui
