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

/** Arrow should display an arrow from the point at the start to the point at the end
 *
 * An ArrowConfig can be provided to specify how the arrow should be displayed
 *
 * \tparam GetStart Returns an Eigen::Vector3d representing the starting point
 *
 * \tparam GetEnd Returns an Eigen::Vector3d representing the end point
 *
 */
template<typename GetStart, typename GetEnd>
struct ArrowROImpl : public Element
{
  static constexpr auto type = Elements::Arrow;

  ArrowROImpl(const std::string & name, const ArrowConfig & config, GetStart get_start_fn, GetEnd get_end_fn)
  : Element(name), get_start_fn_(get_start_fn), get_end_fn_(get_end_fn), config_(config)
  {
    static_assert(details::CheckReturnType<GetStart, Eigen::Vector3d>::value,
                  "Arrow element start callback must return an Eigen::Vector3d");
    static_assert(details::CheckReturnType<GetEnd, Eigen::Vector3d>::value,
                  "Arrow element end callback must return an Eigen::Vector3d");
  }

  /** Invalid element */
  ArrowROImpl(){};

  constexpr static size_t write_size()
  {
    return Element::write_size() + 3 + ArrowConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder, bool ro = true)
  {
    Element::write(builder);
    builder.write(get_start_fn_());
    builder.write(get_end_fn_());
    builder.write(ro);
    config_.write(builder);
  }

private:
  GetStart get_start_fn_;
  GetEnd get_end_fn_;
  ArrowConfig config_;
};

template<typename GetStart, typename SetStart, typename GetEnd, typename SetEnd>
struct ArrowImpl : public ArrowROImpl<GetStart, GetEnd>
{
  ArrowImpl(const std::string & name,
            const ArrowConfig & config,
            GetStart get_start_fn,
            SetStart set_start_fn,
            GetEnd get_end_fn,
            SetEnd set_end_fn)
  : ArrowROImpl<GetStart, GetEnd>(name, config, get_start_fn, get_end_fn), set_start_fn_(set_start_fn),
    set_end_fn_(set_end_fn)
  {
  }

  /** Invalid element */
  ArrowImpl(){};

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    ArrowROImpl<GetStart, GetEnd>::write(builder, false);
  }

  bool handleRequest(const mc_rtc::Configuration & data)
  {
    const Eigen::Vector6d & arrow = data;
    set_start_fn_(arrow.head<3>());
    set_end_fn_(arrow.tail<3>());
    return true;
  }

private:
  SetStart set_start_fn_;
  SetEnd set_end_fn_;
};

/** Helper function to create an ArrowImpl */
template<typename GetStart, typename GetEnd>
ArrowROImpl<GetStart, GetEnd> Arrow(const std::string & name, GetStart get_start_fn, GetEnd get_end_fn)
{
  return ArrowROImpl<GetStart, GetEnd>(name, {}, get_start_fn, get_end_fn);
}

/** Helper function to create an ArrowImpl */
template<typename GetStart, typename GetEnd>
ArrowROImpl<GetStart, GetEnd> Arrow(const std::string & name,
                                    const ArrowConfig & config,
                                    GetStart get_start_fn,
                                    GetEnd get_end_fn)
{
  return ArrowROImpl<GetStart, GetEnd>(name, config, get_start_fn, get_end_fn);
}

template<typename GetStart, typename SetStart, typename GetEnd, typename SetEnd>
ArrowImpl<GetStart, SetStart, GetEnd, SetEnd> Arrow(const std::string & name,
                                                    GetStart get_start_fn,
                                                    SetStart set_start_fn,
                                                    GetEnd get_end_fn,
                                                    SetEnd set_end_fn)
{
  return ArrowImpl<GetStart, SetStart, GetEnd, SetEnd>(name, ArrowConfig{}, get_start_fn, set_start_fn, get_end_fn,
                                                       set_end_fn);
}

template<typename GetStart, typename SetStart, typename GetEnd, typename SetEnd>
ArrowImpl<GetStart, SetStart, GetEnd, SetEnd> Arrow(const std::string & name,
                                                    const ArrowConfig & config,
                                                    GetStart get_start_fn,
                                                    SetStart set_start_fn,
                                                    GetEnd get_end_fn,
                                                    SetEnd set_end_fn)
{
  return ArrowImpl<GetStart, SetStart, GetEnd, SetEnd>(name, config, get_start_fn, set_start_fn, get_end_fn,
                                                       set_end_fn);
}

} // namespace gui

} // namespace mc_rtc
