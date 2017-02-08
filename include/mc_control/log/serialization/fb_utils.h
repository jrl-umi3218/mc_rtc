#pragma once

#include "MCLog_generated.h"

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_control
{

namespace log
{

/** This structure allows to convert native C++ types to their flatbuffer
 * counterpart */
template<typename T>
struct LogDataHelper
{
  /** Holds the value type corresponding to the C++ type */
  enum { value_type = mc_control::log::LogData_NONE };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
  /** Convert the C++ object to an anonymous flatbuffer object */
  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder &,
                                             const T &)
  {
    static_assert(sizeof(T) == 0, "This type is not handled by the logger interface");
  }
#pragma GCC diagnostic pop
};

/** This macro allows to define implementations for LogDataHelper in
 * simple cases */
#define IMPL_LDH(TYPE, LD_TYPE, S_FN, ARG_NAME, ...)\
template<>\
struct LogDataHelper<TYPE>\
{\
  enum { value_type = mc_control::log::LD_TYPE };\
  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder & builder,\
                                             const TYPE & ARG_NAME)\
  {\
    return mc_control::log::S_FN(builder, __VA_ARGS__).Union();\
  }\
}

IMPL_LDH(bool, LogData_Bool, CreateBool, b, b);
IMPL_LDH(double, LogData_Double, CreateDouble, d, d);
IMPL_LDH(std::vector<double>, LogData_DoubleVector, CreateDoubleVectorDirect, v, &v);
IMPL_LDH(unsigned int, LogData_UnsignedInt, CreateUnsignedInt, i, i);
IMPL_LDH(std::string, LogData_String, CreateStringDirect, s, s.c_str());
IMPL_LDH(Eigen::Vector3d, LogData_Vector3d, CreateVector3d, v, v.x(), v.y(), v.z());
IMPL_LDH(Eigen::Quaterniond, LogData_Quaterniond, CreateQuaterniond, q, q.w(), q.x(), q.y(), q.z());

#undef IMPL_DATA_HELPER

template<>
struct LogDataHelper<sva::PTransformd>
{
  enum { value_type = mc_control::log::LogData_PTransformd };

  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder & builder,
                                             const sva::PTransformd & pt)
  {
    auto q = Eigen::Quaterniond(pt.rotation());
    auto fb_q = mc_control::log::CreateQuaterniond(builder, q.w(), q.x(), q.y(), q.z());
    const auto & t = pt.translation();
    auto fb_t = mc_control::log::CreateVector3d(builder, t.x(), t.y(), t.z());
    return mc_control::log::CreatePTransformd(builder, fb_q, fb_t).Union();
  }
};

template<>
struct LogDataHelper<sva::ForceVecd>
{
  enum { value_type = mc_control::log::LogData_ForceVecd };

  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder & builder,
                                             const sva::ForceVecd & fv)
  {
    const auto & couple = fv.couple();
    auto fb_couple = mc_control::log::CreateVector3d(builder, couple.x(), couple.y(), couple.z());
    const auto & force = fv.force();
    auto fb_force = mc_control::log::CreateVector3d(builder, force.x(), force.y(), force.z());
    return mc_control::log::CreateForceVecd(builder, fb_couple, fb_force).Union();
  }
};

template<typename T>
void AddLogData(flatbuffers::FlatBufferBuilder & builder,
                std::vector<uint8_t> & value_types,
                std::vector<flatbuffers::Offset<void>> & values,
                const T & value)
{
  value_types.push_back(LogDataHelper<T>::value_type);
  values.push_back(LogDataHelper<T>::serialize(builder, value));
}

}

}
