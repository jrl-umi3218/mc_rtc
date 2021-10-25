/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/** This file contains some utility template metaprogramming functions for the GUI */

#include <type_traits>

namespace mc_rtc
{

namespace gui
{

namespace details
{

/** Helper for \ref is_getter */
template<typename GetT>
constexpr bool is_getter_impl(decltype(std::declval<GetT>()()) *)
{
  return !std::is_same<decltype(std::declval<GetT>()()), void>::value;
}

template<typename GetT>
constexpr bool is_getter_impl(...)
{
  return false;
}

/** This traits is true if:
 *
 * - \tparam GetT is a nullary functor
 * - \tparam GetT functor returns a non-void value
 */
template<typename GetT>
constexpr bool is_getter()
{
  return is_getter_impl<GetT>(nullptr);
}

/** Get the return type of a getter function */
template<typename GetT>
struct ReturnType
{
  using type = typename std::decay<decltype(std::declval<GetT>()())>::type;
};

/** Helper */
template<typename GetT>
using ReturnTypeT = typename ReturnType<GetT>::type;

/** Check the return type of a getter function
 *
 * value is true if GetT() returns one of the provided argument types
 */
template<typename GetT, typename... Args>
struct CheckReturnType
{
  static constexpr bool value = false;
};

template<typename GetT, typename T>
struct CheckReturnType<GetT, T>
{
  static constexpr bool value = std::is_convertible<ReturnTypeT<GetT>, typename std::decay<T>::type>::value;
};

template<typename GetT, typename T, typename... Args>
struct CheckReturnType<GetT, T, Args...>
{
  static constexpr bool value = CheckReturnType<GetT, T>::value || CheckReturnType<GetT, Args...>::value;
};

} // namespace details

} // namespace gui

} // namespace mc_rtc
