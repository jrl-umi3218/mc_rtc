/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <memory>
#include <type_traits>

namespace mc_rtc
{

/** This template helper allow to enable implicit conversion from instances of
 * T (resp. const T) to std::shared_ptr<T> (resp. std::shared_ptr<const T>).
 *
 * This allows mc_rtc to expose reference (or const reference) to internal
 * objects stored as pointer which are often manipulated without requiring
 * ownership or access a "nested" object without incurring extra pointer copies
 * (e.g. accessing a frame of a robot).
 *
 * \tparam T The type that should be shared
 *
 * \tparam BaseT If provided BaseT must inherit shared<BaseT>. shared<T, BaseT> then inherits BaseT with added
 * conversions for T
 */
template<typename T, typename BaseT = void>
struct shared : public std::conditional<std::is_same<BaseT, void>::value, std::enable_shared_from_this<T>, BaseT>::type
{
  static_assert(std::is_same<BaseT, void>::value || std::is_base_of<shared<BaseT>, BaseT>::value,
                "shared<T, BaseT> requires a void base or a base that derives from shared<BaseT>");
  operator std::shared_ptr<T>()
  {
    return std::static_pointer_cast<T>(static_cast<T *>(this)->shared_from_this());
  }

  operator std::shared_ptr<const T>() const
  {
    return std::static_pointer_cast<const T>(static_cast<const T *>(this)->shared_from_this());
  }

  /** Inherit constructors from the base */
  using ActualBase =
      typename std::conditional<std::is_same<BaseT, void>::value, std::enable_shared_from_this<T>, BaseT>::type;
  using ActualBase::ActualBase;
};

} // namespace mc_rtc
