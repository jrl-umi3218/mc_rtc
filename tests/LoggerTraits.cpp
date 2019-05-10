/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/log/Logger.h>

/** This executable only test the various traits used to discriminate
 * valid logger callbacks, compilation failure would indicate an issue
 * with said traits. */

template<typename T, bool expected>
void test_traits()
{
  constexpr bool b = mc_rtc::log::is_serializable<T>::value;
  static_assert(b == expected, "Unexpected");
  constexpr bool b_value_ret = mc_rtc::log::callback_is_serializable<std::function<T()>>::value;
  static_assert(b_value_ret == expected, "Unexpected");
  constexpr bool b_ref_ret = mc_rtc::log::callback_is_serializable<std::function<T &()>>::value;
  static_assert(b_ref_ret == expected, "Unexpected");
  constexpr bool b_cref_ret = mc_rtc::log::callback_is_serializable<std::function<const T &()>>::value;
  static_assert(b_cref_ret == expected, "Unexpected");
}

int main()
{
  test_traits<bool, true>();
  test_traits<double, true>();
  test_traits<std::vector<double>, true>();
  test_traits<unsigned int, true>();
  test_traits<std::string, true>();
  test_traits<Eigen::Vector3d, true>();
  test_traits<Eigen::Quaterniond, true>();
  test_traits<sva::PTransformd, true>();
  test_traits<sva::ForceVecd, true>();
  test_traits<Eigen::Vector2d, true>();
  test_traits<Eigen::Vector6d, true>();
  test_traits<std::vector<unsigned int>, false>();
  test_traits<float, true>();
  test_traits<std::map<std::string, double>, false>();
  return 0;
}
