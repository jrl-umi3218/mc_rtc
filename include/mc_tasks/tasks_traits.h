/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <Eigen/Core>

#include <type_traits>

/** Meta-programming helpers to detect some function capabilities */

namespace mc_tasks
{

namespace details
{

#define MAKE_GETTER_DETECTOR(GETTER)                                                                         \
  namespace impl                                                                                             \
  {                                                                                                          \
  template<typename T>                                                                                       \
  struct GETTER##_getter_traits                                                                              \
  {                                                                                                          \
    template<typename U>                                                                                     \
    static constexpr std::true_type test_has(std::void_t<decltype(std::declval<U>().GETTER())> * = nullptr); \
    template<typename U>                                                                                     \
    static constexpr std::false_type test_has(...);                                                          \
    static constexpr bool has = decltype(test_has<T>(0))::value;                                             \
    template<typename U>                                                                                     \
    static auto test(std::void_t<decltype(std::declval<const U>().GETTER())> * = nullptr)                    \
        -> decltype(std::declval<const U>().GETTER());                                                       \
    template<typename U>                                                                                     \
    static void test(...);                                                                                   \
    using type = decltype(test<T>(0));                                                                       \
  };                                                                                                         \
  }                                                                                                          \
  template<typename T>                                                                                       \
  inline constexpr bool has_##GETTER##_getter_v = impl::GETTER##_getter_traits<T>::has;                      \
  template<typename T>                                                                                       \
  using GETTER##_return_t = typename impl::GETTER##_getter_traits<T>::type

#define MAKE_SETTER_DETECTOR(SETTER)                                                                   \
  namespace impl                                                                                       \
  {                                                                                                    \
  template<typename T>                                                                                 \
  struct has_##SETTER##_setter                                                                         \
  {                                                                                                    \
    template<typename U>                                                                               \
    static constexpr std::true_type test(                                                              \
        std::void_t<decltype(std::declval<U>().SETTER(std::declval<Eigen::VectorXd>()))> * = nullptr); \
    template<typename U>                                                                               \
    static constexpr std::false_type test(...);                                                        \
    static constexpr bool value = decltype(test<T>(0))::value;                                         \
  };                                                                                                   \
  }                                                                                                    \
  template<typename T>                                                                                 \
  inline constexpr bool has_##SETTER##_setter_v = impl::has_##SETTER##_setter<T>::value

MAKE_GETTER_DETECTOR(refVel);
MAKE_SETTER_DETECTOR(refVel);
MAKE_GETTER_DETECTOR(refAccel);
MAKE_SETTER_DETECTOR(refAccel);

template<typename T>
inline constexpr bool has_refVel_v = has_refVel_getter_v<T> && has_refVel_setter_v<T>;

template<typename T>
inline constexpr bool has_refAccel_v = has_refAccel_getter_v<T> && has_refAccel_setter_v<T>;

template<typename T>
inline constexpr bool always_false_v = false;

#undef MAKE_GETTER_DETECTOR
#undef MAKE_SETTER_DETECTOR

} // namespace details

} // namespace mc_tasks
