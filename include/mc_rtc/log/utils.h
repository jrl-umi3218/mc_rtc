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
IMPL_MAPPING(mc_rbdyn::Gains2d, Vector2d);
IMPL_MAPPING(mc_rbdyn::Gains3d, Vector3d);
IMPL_MAPPING(mc_rbdyn::Gains6d, Vector6d);

#undef IMPL_MAPPING

template<int N, int _Options, int _MaxRows, int _MaxCols>
struct GetLogType<Eigen::Matrix<double, N, 1, _Options, _MaxRows, _MaxCols>>
{
  static constexpr mc_rtc::log::LogType type = mc_rtc::log::LogType::VectorXd;
};

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
  static constexpr LogType type = GetLogType<T>::type;
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
  static constexpr LogType log_type = is_serializable<base_type>::type;
  static constexpr bool value = is_serializable<base_type>::value;
};

/** For a given type, writes to the log */
template<typename T>
struct LogWriter
{
  static void write(const T & data, mc_rtc::MessagePackBuilder & builder) { builder.write(data); }
};

/** Provide a correspondance from a log type to a C++ type */
template<LogType type>
struct log_type_to_type
{
  static_assert(static_cast<std::underlying_type_t<LogType>>(type)
                    == std::numeric_limits<std::underlying_type_t<LogType>>::max(),
                "This must be specialized for the provided LogType value");
};

#define IMPL_MAPPING(ENUMV, CPPT)         \
  template<>                              \
  struct log_type_to_type<LogType::ENUMV> \
  {                                       \
    using type = CPPT;                    \
  }

IMPL_MAPPING(Bool, bool);
IMPL_MAPPING(Int8_t, int8_t);
IMPL_MAPPING(Int16_t, int16_t);
IMPL_MAPPING(Int32_t, int32_t);
IMPL_MAPPING(Int64_t, int64_t);
IMPL_MAPPING(Uint8_t, uint8_t);
IMPL_MAPPING(Uint16_t, uint16_t);
IMPL_MAPPING(Uint32_t, uint32_t);
IMPL_MAPPING(Uint64_t, uint64_t);
IMPL_MAPPING(Float, float);
IMPL_MAPPING(Double, double);
IMPL_MAPPING(String, std::string);
IMPL_MAPPING(Vector2d, Eigen::Vector2d);
IMPL_MAPPING(Vector3d, Eigen::Vector3d);
IMPL_MAPPING(Vector6d, Eigen::Vector6d);
IMPL_MAPPING(VectorXd, Eigen::VectorXd);
IMPL_MAPPING(Quaterniond, Eigen::Quaterniond);
IMPL_MAPPING(PTransformd, sva::PTransformd);
IMPL_MAPPING(ForceVecd, sva::ForceVecd);
IMPL_MAPPING(MotionVecd, sva::MotionVecd);
IMPL_MAPPING(VectorDouble, std::vector<double>);

#undef IMPL_MAPPING

template<LogType type>
using log_type_to_type_t = typename log_type_to_type<type>::type;

} // namespace log

} // namespace mc_rtc
