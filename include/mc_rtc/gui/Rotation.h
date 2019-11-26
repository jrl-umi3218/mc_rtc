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

/** Rotation display a widget that shows the rotation
 *
 * This rotation is not editable.
 *
 * This will also create a quaternion ArrayLabel with labels {"qw", "qx", "qy", "qz"}
 *
 * \tparam GetT Must return an sva::PTransformd (to display the rotation somewhere)
 */
template<typename GetT>
struct RotationROImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Rotation;

  RotationROImpl(const std::string & name, GetT get_fn) : DataElement<GetT>(name, get_fn)
  {
    static_assert(details::CheckReturnType<GetT, sva::PTransformd>::value,
                  "RotationROImpl getter should return an sva::PTransformd");
  }

  constexpr static size_t write_size()
  {
    return DataElement<GetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    DataElement<GetT>::write(builder);
    builder.write(true); // Is read-only
  }

  /** Invalid element */
  RotationROImpl() {}
};

/** Rotation display a widget that shows the rotation
 *
 * This rotation is editable.
 *
 * This will also create a quaternion ArrayInput with labels {"qw", "qx", "qy", "qz"}
 *
 * \tparam GetT Must return an sva::PTransformd (to display the rotation somewhere)
 *
 * \tparam SetT Should accept a rotation as an Eigen::Quaterniond
 */
template<typename GetT, typename SetT>
struct RotationImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::Rotation;

  RotationImpl(const std::string & name, GetT get_fn, SetT set_fn) : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn)
  {
    static_assert(details::CheckReturnType<GetT, sva::PTransformd>::value,
                  "RotationImpl getter should return an sva::PTransformd");
  }

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
  RotationImpl() {}
};

/** Helper function to create a Rotation element (read-only) */
template<typename GetT>
RotationROImpl<GetT> Rotation(const std::string & name, GetT get_fn)
{
  return RotationROImpl<GetT>(name, get_fn);
}

/** Helper function to create a Rotation element (writable) */
template<typename GetT, typename SetT>
RotationImpl<GetT, SetT> Rotation(const std::string & name, GetT get_fn, SetT set_fn)
{
  return RotationImpl<GetT, SetT>(name, get_fn, set_fn);
}

} // namespace gui

} // namespace mc_rtc
