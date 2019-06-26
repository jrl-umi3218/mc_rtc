/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

#include "MCLog_generated.h"

namespace mc_rtc
{

namespace log
{

/** This structure allows to convert native C++ types to their flatbuffer
 * counterpart */
template<typename T>
struct LogDataHelper
{
  /** Holds the value type corresponding to the C++ type */
  static constexpr mc_rtc::log::LogData value_type = mc_rtc::log::LogData_NONE;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
  /** Convert the C++ object to an anonymous flatbuffer object */
  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder &, const T &)
  {
    static_assert(sizeof(T) == 0, "This type is not handled by the logger interface");
  }
#pragma GCC diagnostic pop
};

/** This macro allows to define implementations for LogDataHelper in
 * simple cases */
#define IMPL_LDH(TYPE, LD_TYPE, S_FN, ARG_NAME, ...)                                                            \
  template<>                                                                                                    \
  struct LogDataHelper<TYPE>                                                                                    \
  {                                                                                                             \
    static constexpr mc_rtc::log::LogData value_type = mc_rtc::log::LD_TYPE;                                    \
    static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder & builder, const TYPE & ARG_NAME) \
    {                                                                                                           \
      return mc_rtc::log::S_FN(builder, __VA_ARGS__).Union();                                                   \
    }                                                                                                           \
  }

IMPL_LDH(bool, LogData_Bool, CreateBool, b, b);
IMPL_LDH(double, LogData_Double, CreateDouble, d, d);
IMPL_LDH(std::vector<double>, LogData_DoubleVector, CreateDoubleVectorDirect, v, &v);
IMPL_LDH(unsigned int, LogData_UnsignedInt, CreateUnsignedInt, i, i);
IMPL_LDH(uint64_t, LogData_UInt64, CreateUInt64, i, i);
IMPL_LDH(std::string, LogData_String, CreateStringDirect, s, s.c_str());
IMPL_LDH(Eigen::Vector2d, LogData_Vector2d, CreateVector2d, v, v.x(), v.y());
IMPL_LDH(Eigen::Vector3d, LogData_Vector3d, CreateVector3d, v, v.x(), v.y(), v.z());
IMPL_LDH(Eigen::Quaterniond, LogData_Quaterniond, CreateQuaterniond, q, q.w(), q.x(), q.y(), q.z());

#undef IMPL_DATA_HELPER

template<>
struct LogDataHelper<sva::PTransformd>
{
  static constexpr mc_rtc::log::LogData value_type = mc_rtc::log::LogData_PTransformd;

  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder & builder, const sva::PTransformd & pt)
  {
    auto q = Eigen::Quaterniond(pt.rotation());
    auto fb_q = mc_rtc::log::CreateQuaterniond(builder, q.w(), q.x(), q.y(), q.z());
    const auto & t = pt.translation();
    auto fb_t = mc_rtc::log::CreateVector3d(builder, t.x(), t.y(), t.z());
    return mc_rtc::log::CreatePTransformd(builder, fb_q, fb_t).Union();
  }
};

template<>
struct LogDataHelper<sva::ForceVecd>
{
  static constexpr mc_rtc::log::LogData value_type = mc_rtc::log::LogData_ForceVecd;

  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder & builder, const sva::ForceVecd & fv)
  {
    const auto & couple = fv.couple();
    auto fb_couple = mc_rtc::log::CreateVector3d(builder, couple.x(), couple.y(), couple.z());
    const auto & force = fv.force();
    auto fb_force = mc_rtc::log::CreateVector3d(builder, force.x(), force.y(), force.z());
    return mc_rtc::log::CreateForceVecd(builder, fb_couple, fb_force).Union();
  }
};

template<>
struct LogDataHelper<sva::MotionVecd>
{
  static constexpr mc_rtc::log::LogData value_type = mc_rtc::log::LogData_MotionVecd;

  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder & builder, const sva::MotionVecd & mv)
  {
    const auto & angular = mv.angular();
    auto fb_angular = mc_rtc::log::CreateVector3d(builder, angular.x(), angular.y(), angular.z());
    const auto & linear = mv.linear();
    auto fb_linear = mc_rtc::log::CreateVector3d(builder, linear.x(), linear.y(), linear.z());
    return mc_rtc::log::CreateMotionVecd(builder, fb_angular, fb_linear).Union();
  }
};

template<>
struct LogDataHelper<Eigen::VectorXd>
{
  static constexpr mc_rtc::log::LogData value_type = mc_rtc::log::LogData_DoubleVector;

  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder & builder, const Eigen::VectorXd & v)
  {
    auto fb_v = builder.CreateVector<double>(v.data(), v.size());
    return mc_rtc::log::CreateDoubleVector(builder, fb_v).Union();
  }
};

/** Type-traits for serializable types
 *
 * value is true if T is serializable
 *
 */
template<typename T>
struct is_serializable
{
  static constexpr bool value = LogDataHelper<T>::value_type != mc_rtc::log::LogData_NONE;
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

/** Type-traits to recognize a vector */
template<typename T>
struct is_vector
{
  static constexpr bool value = false;
};

template<typename T>
struct is_vector<std::vector<T>>
{
  static constexpr bool value = true;
};

/** Type-traits to recognize a callable object that returns a const reference
 * to a vector */
template<typename T>
struct callback_returns_crv
{
  using ret_type = typename std::result_of<T()>::type;
  using base_type = typename std::decay<ret_type>::type;
  static constexpr bool value = !is_serializable<base_type>::value && is_vector<base_type>::value
                                && std::is_reference<ret_type>::value
                                && std::is_const<typename std::remove_reference<ret_type>::type>::value;
};

/** Type-traits for callables that returns a const reference to a vector of a
 * serializable types
 *
 * value is true if T() returns a const reference to a vector of a serializable
 * type
 *
 */
template<typename T, typename enable = void>
struct callback_is_crv_of_serializable
{
  static constexpr bool value = false;
};

template<typename T>
struct callback_is_crv_of_serializable<T, typename std::enable_if<callback_returns_crv<T>::value>::type>
{
  static constexpr bool value = is_serializable<typename callback_returns_crv<T>::base_type::value_type>::value;
};

template<typename T, typename = typename std::enable_if<is_serializable<T>::value>::type>
void AddLogData(flatbuffers::FlatBufferBuilder & builder,
                std::vector<uint8_t> & value_types,
                std::vector<flatbuffers::Offset<void>> & values,
                const T & value)
{
  value_types.push_back(LogDataHelper<T>::value_type);
  values.push_back(LogDataHelper<T>::serialize(builder, value));
}

} // namespace log

} // namespace mc_rtc
