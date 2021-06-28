#pragma once

#include <mc_rtc/MessagePackBuilder.h>

namespace mc_rtc
{

namespace log
{

/** Enum representing the type of data being logged */
enum class LogType
{
  None = 0,
  Bool,
  Int8_t,
  Int16_t,
  Int32_t,
  Int64_t,
  Uint8_t,
  Uint16_t,
  Uint32_t,
  Uint64_t,
  Float,
  Double,
  String,
  Vector2d,
  Vector3d,
  Vector6d,
  VectorXd,
  Quaterniond,
  PTransformd,
  ForceVecd,
  MotionVecd,
  VectorDouble
};

inline const char ** LogTypeNames()
{
  static const char * names[] = {"None",        "Bool",      "Int8_t",     "Int16_t",      "Int32_t",  "Int64_t",
                                 "Uint8_t",     "Uint16_t",  "Uint32_t",   "Uint64_t",     "Float",    "Double",
                                 "String",      "Vector2d",  "Vector3d",   "Vector6d",     "VectorXd", "Quaterniond",
                                 "PTransformd", "ForceVecd", "MotionVecd", "VectorDouble", nullptr};
  return names;
}

inline const char * LogTypeName(LogType t)
{
  const size_t idx = static_cast<size_t>(t);
  return LogTypeNames()[idx];
}

/** Helper to build a correspondance between C++ types and enum values */
template<typename T>
struct GetLogType
{
  static constexpr mc_rtc::log::LogType type = mc_rtc::log::LogType::None;
};

#define IMPL_MAPPING(CPPT, ENUMV)                                             \
  template<>                                                                  \
  struct GetLogType<CPPT>                                                     \
  {                                                                           \
    static constexpr mc_rtc::log::LogType type = mc_rtc::log::LogType::ENUMV; \
  }

IMPL_MAPPING(bool, Bool);
IMPL_MAPPING(int8_t, Int8_t);
IMPL_MAPPING(int16_t, Int16_t);
IMPL_MAPPING(int32_t, Int32_t);
IMPL_MAPPING(int64_t, Int64_t);
IMPL_MAPPING(uint8_t, Uint8_t);
IMPL_MAPPING(uint16_t, Uint16_t);
IMPL_MAPPING(uint32_t, Uint32_t);
IMPL_MAPPING(uint64_t, Uint64_t);
IMPL_MAPPING(float, Float);
IMPL_MAPPING(double, Double);
IMPL_MAPPING(std::string, String);
IMPL_MAPPING(Eigen::Vector2d, Vector2d);
IMPL_MAPPING(Eigen::Vector3d, Vector3d);
IMPL_MAPPING(Eigen::Vector6d, Vector6d);
IMPL_MAPPING(Eigen::VectorXd, VectorXd);
IMPL_MAPPING(Eigen::Quaterniond, Quaterniond);
IMPL_MAPPING(sva::PTransformd, PTransformd);
IMPL_MAPPING(sva::ForceVecd, ForceVecd);
IMPL_MAPPING(sva::MotionVecd, MotionVecd);
IMPL_MAPPING(sva::ImpedanceVecd, MotionVecd);

#undef IMPL_MAPPING

template<typename A>
struct GetLogType<std::vector<double, A>>
{
  static constexpr mc_rtc::log::LogType type = mc_rtc::log::LogType::VectorDouble;
};

template<std::size_t N>
struct GetLogType<std::array<double, N>>
{
  static constexpr mc_rtc::log::LogType type = mc_rtc::log::LogType::VectorDouble;
};

template<typename Type, int Options, typename StrideType>
struct GetLogType<Eigen::Ref<Type, Options, StrideType>>
{
  // clang-format off
  static constexpr mc_rtc::log::LogType type =
    Type::ColsAtCompileTime != 1 ? mc_rtc::log::LogType::None :
    Type::RowsAtCompileTime == 2 ? mc_rtc::log::LogType::Vector2d :
    Type::RowsAtCompileTime == 3 ? mc_rtc::log::LogType::Vector3d :
    Type::RowsAtCompileTime == 6 ? mc_rtc::log::LogType::Vector6d :
                                   mc_rtc::log::LogType::VectorXd;
  // clang-format on
};

/** True if the given type is serializable in the log */
template<typename T>
struct is_serializable
{
  static constexpr bool value = GetLogType<T>::type != mc_rtc::log::LogType::None;
};

template<typename...>
using void_t = void;

/** True for member pointer that are serializable */
template<typename T>
struct is_serializable_member
{
  static constexpr bool value = false;
};

template<typename T, typename MemberT>
struct is_serializable_member<MemberT T::*>
{
  static constexpr bool value = is_serializable<typename std::decay<MemberT>::type>::value;
};

template<typename T>
struct is_serializable_getter
{
  static constexpr bool value = false;
};

template<typename T, typename MethodRetT>
struct is_serializable_getter<MethodRetT (T::*)() const>
{
  static constexpr bool value = is_serializable<typename std::decay<MethodRetT>::type>::value;
};

/** Type-traits for callables that returns a serializable type
 *
 *  value is true if T() return type (after decaying) is serializable
 *
 */
template<typename T, typename = void>
struct callback_is_serializable
{
  static constexpr bool value = false;
};

template<typename T>
struct callback_is_serializable<T, void_t<typename std::result_of<T()>::type>>
{
  using ret_type = typename std::result_of<T()>::type;
  using base_type = typename std::decay<ret_type>::type;
  static constexpr bool value = is_serializable<base_type>::value;
};

/** For a given type, writes to the log */
template<typename T>
struct LogWriter
{
  static void write(const T & data, mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(static_cast<typename std::underlying_type<LogType>::type>(GetLogType<T>::type));
    builder.write(data);
  }
};

} // namespace log

} // namespace mc_rtc
