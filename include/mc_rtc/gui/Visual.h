/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/gui/types.h>

#include <mc_rbdyn/configuration_io.h>
#include "mc_rtc/MessagePackBuilder.h"

namespace mc_rtc
{

namespace gui
{

namespace details
{

/** Visual display shows the provided rbd::parsers::Visual element at the provided location
 *
 * \tparam GetVisual Must return an rbd::parsers::Visual
 *
 * \tparam GetPos Must return an sva::PTransformd or an Eigen::Vector3d
 *
 */
template<typename GetVisual, typename GetPos>
struct VisualImpl : public Element
{
  static constexpr auto type = Elements::Visual;

  VisualImpl(const std::string & name, GetVisual get_visual_fn, GetPos get_pos_fn)
  : Element(name), get_visual_fn_(get_visual_fn), get_pos_fn_(get_pos_fn)
  {
    static_assert(details::CheckReturnType<GetVisual, rbd::parsers::Visual>::value,
                  "Visual element visual callback must return an rbd::parsers::Visual");
    static_assert(details::CheckReturnType<GetPos, sva::PTransformd, Eigen::Vector3d>::value,
                  "Visual element position callback must return an sva::PTransformd or an Eigen::Vector3d");
  }

  /** Invalid element */
  VisualImpl() {}

  constexpr static size_t write_size()
  {
    return Element::write_size() + 2;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    Element::write(builder);
    visual_.add("data", get_visual_fn_());
    builder.write(visual_("data"));
    builder.write(get_pos_fn_());
  }

private:
  GetVisual get_visual_fn_;
  GetPos get_pos_fn_;
  mc_rtc::Configuration visual_;
};

} // namespace details

/** Helper function to create a details::VisualImpl */
template<typename GetVisual, typename GetPos>
details::VisualImpl<GetVisual, GetPos> Visual(const std::string & name, GetVisual get_visual_fn, GetPos get_pos_fn)
{
  return details::VisualImpl<GetVisual, GetPos>(name, get_visual_fn, get_pos_fn);
}

} // namespace gui

} // namespace mc_rtc
