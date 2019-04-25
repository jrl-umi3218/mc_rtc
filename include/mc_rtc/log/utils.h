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
  Vector
};

inline const char ** LogTypeNames()
{
  static const char * names[] = {"None",        "Bool",      "Int8_t",     "Int16_t",  "Int32_t",  "Int64_t",
                                 "Uint8_t",     "Uint16_t",  "Uint32_t",   "Uint64_t", "Float",    "Double",
                                 "String",      "Vector2d",  "Vector3d",   "Vector6d", "VectorXd", "Quaterniond",
                                 "PTransformd", "ForceVecd", "MotionVecd", "Vector",   nullptr};
  return names;
}

inline const char * LogTypeName(LogType t)
{
  const size_t idx = static_cast<typename std::underlying_type<LogType>::type>(t);
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

#undef IMPL_MAPPING

template<typename T, typename A>
struct GetLogType<std::vector<T, A>>
{
  static constexpr mc_rtc::log::LogType type = mc_rtc::log::LogType::Vector;
};

/** True if the given type is serializable in the log */
template<typename T>
struct is_serializable
{
  static constexpr bool value = GetLogType<T>::type != mc_rtc::log::LogType::None;
};

template<typename T, typename A>
struct is_serializable<std::vector<T, A>>
{
  static constexpr bool value = is_serializable<T>::value;
};

/** Type-traits for callables that returns a serializable type
 *
 *  value is true if T() return type (after decaying) is serializable
 *
 */
template<typename T>
struct callback_is_serializable
{
  using ret_type = typename std::result_of<T()>::type;
  using base_type = typename std::decay<ret_type>::type;
  static constexpr bool value = is_serializable<base_type>::value;
};

/** For a given type, writes the header */
template<typename T>
struct LogHeaderWriter
{
  /** How many elements this writes */
  static constexpr size_t size = 2;

  /** Writes the header */
  static void write(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(static_cast<typename std::underlying_type<LogType>::type>(GetLogType<T>::type));
  }
};

/** Specialized LogHeaderWriter for std::vector<T, A> */
template<typename T, typename A>
struct LogHeaderWriter<std::vector<T, A>>
{
  static constexpr size_t size = 1 + LogHeaderWriter<T>::size;

  static void write(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(static_cast<typename std::underlying_type<LogType>::type>(LogType::Vector));
    LogHeaderWriter<T>::write(builder);
  }
};

/** For a given type, writes to the log */
template<typename T>
struct LogWriter
{
  static void write(const T & data, mc_rtc::MessagePackBuilder & builder)
  {
    builder.start_array(LogHeaderWriter<T>::size);
    LogHeaderWriter<T>::write(builder);
    builder.write(data);
    builder.finish_array();
  }
};

} // namespace log

} // namespace mc_rtc
