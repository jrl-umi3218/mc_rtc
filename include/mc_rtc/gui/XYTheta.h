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

/** An XYTheta element represents an oriented point in the XY plane.
 *
 * Altitude can be provided optionally.
 *
 * This element is not editable
 *
 * \tparam GetT Should return a double array of size 3 (x, y, theta) or 4 (x, y, theta, z)
 *
 */
template<typename GetT>
struct XYThetaROImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::XYTheta;

  XYThetaROImpl(const std::string & name, GetT get_fn) : DataElement<GetT>(name, get_fn) {}

  /** Invalid element */
  XYThetaROImpl() {}

  constexpr static size_t write_size()
  {
    return DataElement<GetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    DataElement<GetT>::write(builder);
    builder.write(true); // Is read-only
  }
};

/** An XYTheta element represents an oriented point in the XY plane.
 *
 * Altitude can be provided optionally.
 *
 * This element is editable
 *
 * \tparam GetT Should return a double array of size 3 (x, y, theta) or 4 (x, y, theta, z)
 *
 */
template<typename GetT, typename SetT>
struct XYThetaImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::XYTheta;

  XYThetaImpl(const std::string & name, GetT get_fn, SetT set_fn) : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn) {}

  constexpr static size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CommonInputImpl<GetT, SetT>::write(builder);
    builder.write(false); // Is read-only
  }

  /** Invalid element */
  XYThetaImpl() {}
};

/** Helper function to create an XYTheta element (read-only) */
template<typename GetT>
XYThetaROImpl<GetT> XYTheta(const std::string & name, GetT get_fn)
{
  return XYThetaROImpl<GetT>(name, get_fn);
}

/** Helper function to create an XYTheta element */
template<typename GetT, typename SetT>
XYThetaImpl<GetT, SetT> XYTheta(const std::string & name, GetT get_fn, SetT set_fn)
{
  return XYThetaImpl<GetT, SetT>(name, get_fn, set_fn);
}

} // namespace gui

} // namespace mc_rtc
