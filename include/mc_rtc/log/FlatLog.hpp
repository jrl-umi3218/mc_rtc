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
  CPPT const * record_cast(const FlatLog::record & r)                                                 \
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
std::vector<T const *> FlatLog::get(const std::string & entry) const
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

} // namespace log

} // namespace mc_rtc
