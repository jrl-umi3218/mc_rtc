/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/plot/types.h>

namespace mc_rtc
{

namespace gui
{

namespace plot
{

namespace impl
{

/** This holds information about multiple polygons */
template<typename GetT>
struct Polygons
{
  static constexpr Type type = Type::Polygons;

  Polygons(const std::string & name, GetT get_fn, Side side) : name_(name), get_fn_(get_fn), side_(side)
  {
    static_assert(details::CheckReturnType<GetT, std::vector<PolygonDescription>>::value,
                  "Polygons element callback must return a vector of PolygonDescription");
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.start_array(4);
    builder.write(static_cast<uint64_t>(type));
    builder.write(name_);
    const auto & polygons = get_fn_();
    builder.start_array(polygons.size());
    for(const auto & poly : polygons)
    {
      poly.write(builder);
    }
    builder.finish_array();
    builder.write(static_cast<uint64_t>(side_));
    builder.finish_array();
  }

  Polygons & side(Side side)
  {
    side_ = side;
    return *this;
  }

private:
  std::string name_;
  GetT get_fn_;
  Side side_;
};

} // namespace impl

/** Helper function to create an impl::Polygon */
template<typename GetT>
impl::Polygons<GetT> Polygons(const std::string & name, GetT get_fn, Side side = Side::Left)
{
  return impl::Polygons<GetT>(name, get_fn, side);
}

} // namespace plot

} // namespace gui

} // namespace mc_rtc
