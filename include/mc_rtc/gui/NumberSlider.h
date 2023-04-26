/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc::gui
{

namespace details
{

/** NumberSlider should display a number slider with minimum
 * and maximum values specified
 *
 * \tparam GetT Returns a numeric type
 *
 * \tparam SetT Called when the slider is manipulated by the
 * client
 */
template<typename GetT, typename SetT>
struct NumberSliderImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::NumberSlider;

  NumberSliderImpl(const std::string & name, GetT get_fn, SetT set_fn, double min, double max)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn), min_(min), max_(max)
  {
  }

  NumberSliderImpl() {}

  static constexpr size_t write_size() { return CommonInputImpl<GetT, SetT>::write_size() + 2; }

  void write(mc_rtc::MessagePackBuilder & writer)
  {
    CommonInputImpl<GetT, SetT>::write(writer);
    writer.write(min_);
    writer.write(max_);
  }

private:
  double min_;
  double max_;
};

} // namespace details

/** Helper function to create a NumberSliderImpl */
template<typename GetT, typename SetT>
auto NumberSlider(const std::string & name, GetT get_fn, SetT set_fn, double min, double max)
{
  return details::NumberSliderImpl(name, get_fn, set_fn, min, max);
}

/** Helper function to create a NumberSlider from a variable */
template<typename T>
auto NumberSlider(const std::string & name, T & value, double min, double max)
{
  return details::NumberSliderImpl(name, details::read(value), details::write(value), min, max);
}

} // namespace mc_rtc::gui
