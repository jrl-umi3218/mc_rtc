#include <mc_rtc/log/Logger.h>

/** This executable only test the various traits used to discriminate
 * valid logger callbacks, compilation failure would indicate an issue
 * with said traits. */

#define test_is_serializable(T)\
{\
  constexpr bool b = mc_rtc::log::is_serializable<T>::value;\
  static_assert(b, "Type "#T" is not serializable");\
  constexpr bool b_value_ret = mc_rtc::log::callback_is_serializable<std::function<T()>>::value;\
  static_assert(b_value_ret, "A function returning "#T" by value could not be serialized");\
  constexpr bool b_ref_ret = mc_rtc::log::callback_is_serializable<std::function<T&()>>::value;\
  static_assert(b_ref_ret, "A function returning "#T" by reference could not be serialized");\
  constexpr bool b_cref_ret = mc_rtc::log::callback_is_serializable<std::function<const T&()>>::value;\
  static_assert(b_cref_ret, "A function returning "#T" by const reference could not be serialized");\
  constexpr bool b_vector_is_serializable = mc_rtc::log::is_serializable<std::vector<T>>::value;\
  constexpr bool b_vector_cref = !b_vector_is_serializable && mc_rtc::log::callback_is_crv_of_serializable<std::function<const std::vector<T>&()>>::value;\
  static_assert(b_vector_cref, "A function returning a const reference to a vector of "#T" could not be serialized");\
}

#define test_is_not_serializable(T)\
{\
  constexpr bool b = mc_rtc::log::is_serializable<T>::value;\
  static_assert(!b, "Type "#T" is serializable");\
  constexpr bool b_value_ret = mc_rtc::log::callback_is_serializable<std::function<T()>>::value;\
  static_assert(!b_value_ret, "A function returning "#T" by value could be serialized");\
}

template<typename T, bool expected>
void test_traits()
{
  constexpr bool b = mc_rtc::log::is_serializable<T>::value;
  static_assert(b == expected, "Unexpected");
  constexpr bool b_value_ret = mc_rtc::log::callback_is_serializable<std::function<T()>>::value;
  static_assert(b_value_ret == expected, "Unexpected");
  constexpr bool b_ref_ret = mc_rtc::log::callback_is_serializable<std::function<T&()>>::value;
  static_assert(b_ref_ret == expected, "Unexpected");
  constexpr bool b_cref_ret = mc_rtc::log::callback_is_serializable<std::function<const T&()>>::value;
  static_assert(b_cref_ret == expected, "Unexpected");
  constexpr bool b_vector_is_serializable = mc_rtc::log::is_serializable<std::vector<T>>::value;
  constexpr bool b_vector_cref = (b && b_vector_is_serializable) || mc_rtc::log::callback_is_crv_of_serializable<std::function<const std::vector<T>&()>>::value;
  static_assert(b_vector_cref == expected, "Unexpected");
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
  test_traits<Eigen::Vector2d, false>();
  test_traits<Eigen::Vector6d, false>();
  test_traits<std::vector<unsigned int>, false>();
  test_traits<float, false>();
  return 0;
}
