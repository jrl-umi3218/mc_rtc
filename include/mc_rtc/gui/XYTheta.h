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

/** An XYTheta element represents an oriented point in the XY plane.
 *
 * Altitude can be provided optionally.
 *
 * This element is editable if \tparam SetT is not nullptr_t
 *
 * \tparam GetT Should return a double array of size 3nullptr (x, y, theta) or 4 (x, y, theta, z)
 *
 * \tparam SetT Should accept an Eigen::Vector4d that will contain (x, y, theta, z)
 *
 */
template<typename GetT, typename SetT = std::nullptr_t>
struct XYThetaImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::XYTheta;

  XYThetaImpl(const std::string & name, GetT get_fn, SetT set_fn = nullptr)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn)
  {
  }

  constexpr static size_t write_size() { return CommonInputImpl<GetT, SetT>::write_size() + 1; }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CommonInputImpl<GetT, SetT>::write(builder);
    // True for read-only
    builder.write(std::is_same_v<SetT, std::nullptr_t>);
  }

  /** Invalid element */
  XYThetaImpl() {}
};

} // namespace details

/** Helper function to create an XYTheta element (read-only) */
template<typename GetT, std::enable_if_t<std::is_invocable_v<GetT>, int> = 0>
auto XYTheta(const std::string & name, GetT get_fn)
{
  return details::XYThetaImpl(name, get_fn);
}

/** Helper function to create an XYTheta element */
template<typename GetT, typename SetT>
auto XYTheta(const std::string & name, GetT get_fn, SetT set_fn)
{
  return details::XYThetaImpl(name, get_fn, set_fn);
}

} // namespace mc_rtc::gui
