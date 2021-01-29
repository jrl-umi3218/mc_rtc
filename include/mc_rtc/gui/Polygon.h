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

/** Polygon should display a polygon or a set of polygons
 *
 * A color is provided to display the polygon(s)
 *
 * \tparam GetT Should return an std::vector<Eigen::Vector3d> (one polygon) or an
 * std::vector<std::vector<Eigen:Vector3d>> (list of polygons)
 *
 */
template<typename GetT>
struct PolygonImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Polygon;

  PolygonImpl(const std::string & name, const LineConfig & config, GetT get_fn)
  : DataElement<GetT>(name, get_fn), config_(config)
  {
    static_assert(
        details::CheckReturnType<GetT, std::vector<Eigen::Vector3d>, std::vector<std::vector<Eigen::Vector3d>>>::value,
        "Polygon element data callback must return either an std::vector of Eigen::Vector3d or an std::vector of "
        "std::vector3d of Eigen::Vector3d");
  }

  /** Invalid element */
  PolygonImpl() {}

  static constexpr size_t write_size()
  {
    return DataElement<GetT>::write_size() + Color::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    DataElement<GetT>::write(builder);
    config_.write(builder);
  }

private:
  LineConfig config_;
};

/** Helper function to build a PolygonImpl */
template<typename GetT>
PolygonImpl<GetT> Polygon(const std::string & name, GetT get_fn)
{
  return PolygonImpl<GetT>(name, {}, get_fn);
}

/** Helper function to build a PolygonImpl */
template<typename GetT>
PolygonImpl<GetT> Polygon(const std::string & name, const Color & color, GetT get_fn)
{
  return PolygonImpl<GetT>(name, color, get_fn);
}

/** Helper function to build a PolygonImpl */
template<typename GetT>
PolygonImpl<GetT> Polygon(const std::string & name, const LineConfig & config, GetT get_fn)
{
  return PolygonImpl<GetT>(name, config, get_fn);
}

} // namespace gui

} // namespace mc_rtc
