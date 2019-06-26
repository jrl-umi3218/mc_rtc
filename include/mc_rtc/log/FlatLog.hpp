/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/logging.h>

#include "FlatLog.h"

namespace mc_rtc
{

namespace log
{

namespace details
{

template<typename T>
const T * record_cast(const FlatLog::record &)
{
  static_assert(sizeof(T) == 0, "This conversion is not implemented");
}

#define IMPL_RECORD_CAST(ENUM)                                                                               \
  template<>                                                                                                 \
  inline const mc_rtc::log::ENUM * record_cast(const FlatLog::record & r)                                    \
  {                                                                                                          \
    return r.type == mc_rtc::log::LogData_##ENUM ? static_cast<const mc_rtc::log::ENUM *>(r.data) : nullptr; \
  }
IMPL_RECORD_CAST(Bool)
IMPL_RECORD_CAST(Double)
IMPL_RECORD_CAST(DoubleVector)
IMPL_RECORD_CAST(UnsignedInt)
IMPL_RECORD_CAST(String)
IMPL_RECORD_CAST(Vector3d)
IMPL_RECORD_CAST(Quaterniond)
IMPL_RECORD_CAST(PTransformd)
IMPL_RECORD_CAST(ForceVecd)
IMPL_RECORD_CAST(MotionVecd)
IMPL_RECORD_CAST(Vector2d)
IMPL_RECORD_CAST(UInt64)
#undef IMPL_RECORD_CAST

} // namespace details

template<typename T>
std::vector<const T *> FlatLog::getRaw(const std::string & entry) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return {};
  }
  std::vector<T const *> ret;
  const auto & data = at(entry);
  ret.reserve(data.size());
  for(const auto & r : data)
  {
    ret.push_back(details::record_cast<T>(r));
  }
  return ret;
}

namespace details
{

/** Catch all conversion, those invalid conversions don't really happen anyway */
template<typename T, typename LogT>
T convert(const LogT *)
{
  LOG_ERROR_AND_THROW(std::runtime_error,
                      "This should never be called " << __FILE__ << " " << __LINE__ << " " << __PRETTY_FUNCTION__)
  return T{};
}

#define trivial_convert(ENUM, CPPT, MEMBER)                                                                  \
  template<typename T>                                                                                       \
  typename std::enable_if<std::is_convertible<CPPT, T>::value, T>::type convert(const mc_rtc::log::ENUM * v) \
  {                                                                                                          \
    return v->MEMBER();                                                                                      \
  }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wconversion"
trivial_convert(Bool, bool, b);
trivial_convert(Double, double, d);
trivial_convert(UnsignedInt, unsigned int, i);
trivial_convert(UInt64, uint64_t, i);
trivial_convert(String, std::string, s()->str);
#pragma GCC diagnostic pop

#undef trivial_convert

template<>
inline Eigen::Vector2d convert(const mc_rtc::log::Vector2d * v)
{
  return Eigen::Vector2d(v->x(), v->y());
}

template<>
inline Eigen::Vector3d convert(const mc_rtc::log::Vector3d * v)
{
  return Eigen::Vector3d(v->x(), v->y(), v->z());
}

template<>
inline Eigen::Quaterniond convert(const mc_rtc::log::Quaterniond * v)
{
  return Eigen::Quaterniond(v->w(), v->x(), v->y(), v->z());
}

template<>
inline sva::PTransformd convert(const mc_rtc::log::PTransformd * v)
{
  return sva::PTransformd(convert<Eigen::Quaterniond>(v->ori()), convert<Eigen::Vector3d>(v->pos()));
}

template<>
inline sva::ForceVecd convert(const mc_rtc::log::ForceVecd * v)
{
  return sva::ForceVecd(convert<Eigen::Vector3d>(v->couple()), convert<Eigen::Vector3d>(v->force()));
}

template<>
inline sva::MotionVecd convert(const mc_rtc::log::MotionVecd * v)
{
  return sva::MotionVecd(convert<Eigen::Vector3d>(v->angular()), convert<Eigen::Vector3d>(v->linear()));
}

template<>
inline std::vector<double> convert(const mc_rtc::log::DoubleVector * v)
{
  auto ret = std::vector<double>(v->v()->size());
  for(size_t i = 0; i < ret.size(); ++i)
  {
    ret[i] = v->v()->operator[](static_cast<flatbuffers::uoffset_t>(i));
  }
  return ret;
}

template<typename T>
T convert(const FlatLog::record & record)
{
  switch(record.type)
  {
#define IMPL_CASE(ENUM)                                                     \
  case mc_rtc::log::LogData_##ENUM:                                         \
    return convert<T>(static_cast<const mc_rtc::log::ENUM *>(record.data)); \
    break;
    IMPL_CASE(Bool)
    IMPL_CASE(Double)
    IMPL_CASE(DoubleVector)
    IMPL_CASE(UnsignedInt)
    IMPL_CASE(UInt64)
    IMPL_CASE(String)
    IMPL_CASE(Vector2d)
    IMPL_CASE(Vector3d)
    IMPL_CASE(Quaterniond)
    IMPL_CASE(PTransformd)
    IMPL_CASE(ForceVecd)
    IMPL_CASE(MotionVecd)
#undef IMPL_CASE
    default:
      LOG_ERROR_AND_THROW(std::runtime_error, "Attempted to access log data that cannot be converted");
  }
}

template<typename T>
bool convert(const FlatLog::record & record, T & out)
{
  switch(record.type)
  {
#define IMPL_CASE(TRAIT, CPPT, ENUM) \
  case mc_rtc::log::LogData_##ENUM:  \
    if(TRAIT<CPPT, T>::value)        \
    {                                \
      out = convert<T>(record);      \
      return true;                   \
    }                                \
    break;
    IMPL_CASE(std::is_convertible, bool, Bool)
    IMPL_CASE(std::is_convertible, double, Double)
    IMPL_CASE(std::is_convertible, unsigned int, UnsignedInt)
    IMPL_CASE(std::is_convertible, uint64_t, UInt64)
    IMPL_CASE(std::is_same, std::vector<double>, DoubleVector)
    IMPL_CASE(std::is_same, std::string, String)
    IMPL_CASE(std::is_same, Eigen::Vector2d, Vector2d)
    IMPL_CASE(std::is_same, Eigen::Vector3d, Vector3d)
    IMPL_CASE(std::is_same, Eigen::Quaterniond, Quaterniond)
    IMPL_CASE(std::is_same, sva::PTransformd, PTransformd)
    IMPL_CASE(std::is_same, sva::ForceVecd, ForceVecd)
    IMPL_CASE(std::is_same, sva::MotionVecd, MotionVecd)
#undef IMPL_CASE
    default:
      break;
  }
  return false;
}

} // namespace details

template<typename T>
std::vector<T> FlatLog::get(const std::string & entry, const T & def) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return {};
  }
  const auto & data = at(entry);
  std::vector<T> ret(data.size(), def);
  for(size_t i = 0; i < data.size(); ++i)
  {
    details::convert<T>(data[i], ret[i]);
  }
  return ret;
}

template<typename T>
std::vector<T> FlatLog::get(const std::string & entry) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return {};
  }
  const auto & data = at(entry);
  std::vector<T> ret;
  size_t start_i = 0;
  T def{};
  while(start_i < data.size())
  {
    const auto & r = data[start_i];
    if(details::convert<T>(r, def))
    {
      break;
    }
    start_i++;
  }
  if(start_i == data.size())
  {
    LOG_ERROR(entry << " was not logged as the requested data type")
    return ret;
  }
  ret.resize(start_i, def);
  ret.reserve(data.size());
  for(size_t i = start_i; i < data.size(); ++i)
  {
    details::convert<T>(data[i], def);
    ret.push_back(def);
  }
  return ret;
}

template<typename T>
T FlatLog::get(const std::string & entry, size_t i, const T & def) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return def;
  }
  const auto & data = at(entry);
  if(i >= data.size())
  {
    LOG_ERROR("Requested data (" << entry << ") out of available range (" << i << ", available: " << data.size() << ")")
    return def;
  }
  T out = def;
  details::convert<T>(data[i], out);
  return out;
}

} // namespace log

} // namespace mc_rtc
