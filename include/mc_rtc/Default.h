#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

#include <string>
#include <type_traits>
#include <variant>

namespace mc_rtc
{

/** Helper to get a default value for a given type
 *
 * When this is implemented, the template should provide Default<T>::value of type T
 *
 * Enable is used to SFINAE generic definitions
 */
template<typename T, typename Enable = void>
struct Default
{
  static_assert(!std::is_same_v<T, T>, "Must be specialized");
};

template<typename T>
struct Default<T, std::enable_if_t<std::is_arithmetic_v<T>>>
{
  inline static constexpr T value = 0;
};

template<typename Scalar, int N, int Options, int MaxRows, int MaxCols>
struct Default<Eigen::Matrix<Scalar, N, 1, Options, MaxRows, MaxCols>, std::enable_if_t<(N > 0)>>
{
  inline static const Eigen::Matrix<Scalar, N, 1, Options, MaxRows, MaxCols> value =
      Eigen::Matrix<Scalar, N, 1, Options, MaxRows, MaxCols>::Zero();
};

template<typename Scalar, int N, int Options, int MaxRows, int MaxCols>
struct Default<Eigen::Matrix<Scalar, N, N, Options, MaxRows, MaxCols>, std::enable_if_t<(N > 1)>>
{
  inline static const Eigen::Matrix<Scalar, N, N, Options, MaxRows, MaxCols> value =
      Eigen::Matrix<Scalar, N, N, Options, MaxRows, MaxCols>::Identity();
};

template<>
struct Default<sva::PTransformd>
{
  inline static const sva::PTransformd value = sva::PTransformd::Identity();
};

template<>
struct Default<sva::MotionVecd>
{
  inline static const sva::MotionVecd value = sva::MotionVecd::Zero();
};

template<>
struct Default<sva::ForceVecd>
{
  inline static const sva::ForceVecd value = sva::ForceVecd::Zero();
};

template<>
struct Default<sva::ImpedanceVecd>
{
  inline static const sva::ImpedanceVecd value = sva::ImpedanceVecd::Zero();
};

template<>
struct Default<sva::AdmittanceVecd>
{
  inline static const sva::AdmittanceVecd value = sva::AdmittanceVecd::Zero();
};

template<>
struct Default<std::string>
{
  inline static const std::string value;
};

template<typename T, typename... Others>
struct Default<std::variant<T, Others...>> : public Default<T>
{
};

} // namespace mc_rtc
