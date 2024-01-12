/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/** This file contains some utility template metaprogramming functions for the GUI */

#include <mc_rtc/constants.h>

#include <mc_rbdyn/rpy_utils.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <array>
#include <type_traits>
#include <variant>
#include <vector>

namespace mc_rtc::gui::details
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

template<typename CallbackT, typename FunT>
inline constexpr bool has_compatible_signature_v = std::is_convertible_v<CallbackT, std::function<FunT>>;

/** Given a type provides appropriate labels.
 *
 * The following types are supported:
 * - Eigen::Vector2d -> {"x", "y"}
 * - Eigen::Vector3d -> {"x", "y", "z"}
 * - Eigen::Vector4d -> {"x", "y", "z", "w"}
 * - Eigen::Quaterniond -> {"w", "x", "y", "z"}
 * - Eigen::Quaterniond -> {"w", "x", "y", "z"}
 * - sva::MotionVecd -> {"wx", "wy", "wz", "vx", "vy", "vz"}
 * - sva::ForceVecd -> {"cx", "cy", "cz", "fx", "fy", "fz"}
 * - sva::ImpedanceVecd -> {"cx", "cy", "cz", "fx", "fy", "fz"}
 */
template<typename T>
struct Labels
{
  static constexpr bool has_labels = false;
  inline static const std::vector<std::string> labels = {};
};

template<>
struct Labels<Eigen::Vector2d>
{
  static constexpr bool has_labels = true;
  inline static const std::vector<std::string> labels = {"x", "y"};
};

template<>
struct Labels<Eigen::Vector3d>
{
  static constexpr bool has_labels = true;
  inline static const std::vector<std::string> labels = {"x", "y", "z"};
};

template<>
struct Labels<Eigen::Vector4d>
{
  static constexpr bool has_labels = true;
  inline static const std::vector<std::string> labels = {"x", "y", "z", "w"};
};

template<>
struct Labels<Eigen::Quaterniond>
{
  static constexpr bool has_labels = true;
  inline static const std::vector<std::string> labels = {"w", "x", "y", "z"};
};

template<>
struct Labels<sva::MotionVecd>
{
  static constexpr bool has_labels = true;
  inline static const std::vector<std::string> labels = {"wx", "wy", "wz", "vx", "vy", "vz"};
};

template<>
struct Labels<sva::ForceVecd>
{
  static constexpr bool has_labels = true;
  inline static const std::vector<std::string> labels = {"cx", "cy", "cz", "fx", "fy", "fz"};
};

template<>
struct Labels<sva::ImpedanceVecd>
{
  static constexpr bool has_labels = true;
  inline static const std::vector<std::string> labels = {"cx", "cy", "cz", "fx", "fy", "fz"};
};

template<bool Degrees>
struct RPYLabels
{
  inline static const std::vector<std::string> labels = {"r [deg]", "p [deg]", "y [deg]"};
};

template<>
struct RPYLabels<false>
{
  inline static const std::vector<std::string> labels = {"r [rad]", "p [rad]", "y [rad]"};
};

/** Wrap a temporary value into a callback */
template<typename T>
auto read(const T && value)
{
  return [value]() -> const T & { return value; };
}

/** Read RPY angles from a value */
template<bool Degrees, typename T>
auto read_rpy(const T && value)
{
  return [value]() -> Eigen::Vector3d
  {
    if constexpr(Degrees) { return mc_rbdyn::rpyFromRotation(value) * 180. / mc_rtc::constants::PI; }
    else { return mc_rbdyn::rpyFromRotation(value); }
  };
}

/** Wrap a variable into a callback */
template<typename T>
auto read(const T & value)
{
  return [&value]() -> const T & { return value; };
}

/** Read RPY angles from a variable */
template<bool Degrees, typename T>
auto read_rpy(const T & value)
{
  return [&value]() -> Eigen::Vector3d
  {
    if constexpr(Degrees) { return mc_rbdyn::rpyFromRotation(value) * 180. / mc_rtc::constants::PI; }
    else { return mc_rbdyn::rpyFromRotation(value); }
  };
}

/** Make a setter callback for a variable */
template<typename T>
auto write(T & value)
{
  return [&value](const T & v) { value = v; };
}

/** If \tparam T is callable returns value_or_cb() otherwise return value_or_cb */
template<typename T>
auto GetValueOrCallbackValue(const T & value_or_cb)
{
  if constexpr(std::is_invocable_v<T>) { return value_or_cb(); }
  else { return value_or_cb; }
}

/** Type trait to detect a variant */
template<typename T>
struct is_variant : public std::false_type
{
};

template<typename... Args>
struct is_variant<std::variant<Args...>> : public std::true_type
{
};

template<typename T>
inline constexpr bool is_variant_v = is_variant<T>::value;

} // namespace mc_rtc::gui::details
