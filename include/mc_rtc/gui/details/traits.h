/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/** This file contains some utility template metaprogramming functions for the GUI */

#include <array>
#include <type_traits>
#include <vector>

namespace mc_rtc
{

namespace gui
{

namespace details
{

/** Same as std::void_t */
template<typename...>
using void_t = void;

/** Helper for \ref is_getter */
template<typename GetT>
constexpr bool is_getter_impl(void_t<decltype(std::declval<GetT>()())> *)
{
  using ReturnT = typename std::decay<decltype(std::declval<GetT>()())>::type;
  return !std::is_pointer<ReturnT>::value && !std::is_same<ReturnT, void>::value;
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

/** Tag type for non-getter */
struct NotAGetter
{
};

/** Helper for \ref ReturnType */
template<typename GetT, bool is_getter>
struct ReturnTypeImpl
{
  using type = typename std::decay<decltype(std::declval<GetT>()())>::type;
};

template<typename GetT>
struct ReturnTypeImpl<GetT, false>
{
  using type = NotAGetter;
};

/** Get the return type of a getter function */
template<typename GetT>
struct ReturnType
{
  using type = typename ReturnTypeImpl<GetT, is_getter<GetT>()>::type;
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

/** Check whether type is an std::array or std::vector */
template<typename T>
struct is_array_or_vector
{
  enum
  {
    value = false
  };
};

template<typename T, typename A>
struct is_array_or_vector<std::vector<T, A>>
{
  enum
  {
    value = true
  };
};

template<typename T, std::size_t N>
struct is_array_or_vector<std::array<T, N>>
{
  enum
  {
    value = true
  };
};

} // namespace details

} // namespace gui

} // namespace mc_rtc
