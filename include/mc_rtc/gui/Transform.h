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

/** Transform display a widget that shows a 6D frame
 *
 * This element is editable if \tparam SetT is not nullptr_t
 *
 * This will also create an ArrayLabel/ArrayInput with labels {"qw", "qx", "qy",
 * "qz", "tx", "ty", "tz"}
 *
 * \tparam GetT Must return an sva::PTransformd
 *
 * \tparam SetT Should accept an sva::PTransformd
 *
 */
template<typename GetT, typename SetT = std::nullptr_t>
struct TransformImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::Transform;

  TransformImpl(const std::string & name, GetT get_fn, SetT set_fn = nullptr)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn)
  {
    static_assert(details::CheckReturnType<GetT, sva::PTransformd>::value,
                  "TransformImpl getter should return an sva::PTransformd");
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
  TransformImpl() {}
};

} // namespace details

/** Helper function to create a Transform element (read-only) */
template<typename GetT, std::enable_if_t<std::is_invocable_v<GetT>, int> = 0>
auto Transform(const std::string & name, GetT get_fn)
{
  return details::TransformImpl(name, get_fn);
}

/** Helper function to create a Transform element (editable) */
template<typename GetT, typename SetT>
auto Transform(const std::string & name, GetT get_fn, SetT set_fn)
{
  return details::TransformImpl(name, get_fn, set_fn);
}

/** Helper function to create a read-only transform display from a variable */
template<typename T>
auto TransformRO(const std::string & name, T && value)
{
  return Transform(name, details::read(std::forward<T>(value)));
}

/** Helper function to create a writable transform element from a variable */
template<typename T, std::enable_if_t<!std::is_invocable_v<T>, int> = 0>
auto Transform(const std::string & name, T & value)
{
  return Transform(name, details::read(value), details::write(value));
}

} // namespace mc_rtc::gui
