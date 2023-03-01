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

/** Rotation display a widget that shows the rotation
 *
 * This element is editable if \tparam SetT is not nullptr_t
 *
 * This will also create a quaternion ArrayLabel/ArrayInput with labels {"qw", "qx", "qy", "qz"}
 *
 * \tparam GetT Must return an sva::PTransformd (to display the rotation somewhere)
 *
 * \tparam SetT Should accept a rotation as an Eigen::Quaterniond
 */
template<typename GetT, typename SetT = std::nullptr_t>
struct RotationImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::Rotation;

  RotationImpl(const std::string & name, GetT get_fn, SetT set_fn = nullptr)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn)
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
    // True for read-only
    builder.write(std::is_same_v<SetT, std::nullptr_t>);
  }

  /** Invalid element */
  RotationImpl() {}
};

} // namespace details

/** Helper function to create a Rotation element (read-only) */
template<typename GetT>
auto Rotation(const std::string & name, GetT get_fn)
{
  return details::RotationImpl(name, get_fn);
}

/** Helper function to create a Rotation element (writable) */
template<typename GetT, typename SetT>
auto Rotation(const std::string & name, GetT get_fn, SetT set_fn)
{
  return details::RotationImpl(name, get_fn, set_fn);
}

} // namespace mc_rtc::gui
