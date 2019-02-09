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
T const * record_cast(const FlatLog::record &)
{
  static_assert(sizeof(T) == 0, "This conversion is not implemented");
}

#define IMPL_RECORD_CAST(CPPT, ENUM)                                                                  \
  template<>                                                                                          \
  inline CPPT const * record_cast(const FlatLog::record & r)                                          \
  {                                                                                                   \
    return r.type == mc_rtc::log::LogData_##ENUM ? static_cast<CPPT const *>(r.data.get()) : nullptr; \
  }
IMPL_RECORD_CAST(bool, Bool)
IMPL_RECORD_CAST(double, Double)
IMPL_RECORD_CAST(std::vector<double>, DoubleVector)
IMPL_RECORD_CAST(unsigned int, UnsignedInt)
IMPL_RECORD_CAST(std::string, String)
IMPL_RECORD_CAST(Eigen::Vector3d, Vector3d)
IMPL_RECORD_CAST(Eigen::Quaterniond, Quaterniond)
IMPL_RECORD_CAST(sva::PTransformd, PTransformd)
IMPL_RECORD_CAST(sva::ForceVecd, ForceVecd)
IMPL_RECORD_CAST(sva::MotionVecd, MotionVecd)
IMPL_RECORD_CAST(Eigen::Vector2d, Vector2d)
IMPL_RECORD_CAST(uint64_t, UInt64)
#undef IMPL_RECORD_CAST

} // namespace details

template<typename T>
std::vector<T const *> FlatLog::getRaw(const std::string & entry) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return {};
  }
  std::vector<T const *> ret;
  const auto & data = data_.at(entry);
  ret.reserve(data.size());
  for(const auto & r : data)
  {
    ret.push_back(details::record_cast<T>(r));
  }
  return ret;
}

template<typename T>
std::vector<T> FlatLog::get(const std::string & entry, const T & def) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return {};
  }
  const auto & data = data_.at(entry);
  std::vector<T> ret(data.size(), def);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const auto & r = data[i];
    auto ptr = details::record_cast<T>(r);
    if(ptr)
    {
      ret[i] = *ptr;
    }
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
  const auto & data = data_.at(entry);
  std::vector<T> ret;
  size_t start_i = 0;
  while(start_i < data.size())
  {
    const auto & r = data[start_i];
    if(details::record_cast<T>(r))
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
  auto def = std::cref(*details::record_cast<T>(data[start_i]));
  ret.resize(start_i, def);
  ret.reserve(data.size());
  for(size_t i = start_i; i < data.size(); ++i)
  {
    const auto & r = data[i];
    auto ptr = details::record_cast<T>(r);
    if(ptr)
    {
      ret.push_back(*ptr);
      def = *ptr;
    }
    else
    {
      ret.push_back(def);
    }
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
  const auto & data = data_.at(entry);
  if(i >= data.size())
  {
    LOG_ERROR("Requested data (" << entry << ") out of available range (" << i << ", available: " << data.size() << ")")
    return def;
  }
  auto ptr = details::record_cast<T>(data[i]);
  if(ptr)
  {
    return *ptr;
  }
  else
  {
    return def;
  }
}

} // namespace log

} // namespace mc_rtc
