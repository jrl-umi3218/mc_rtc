#pragma once

#include <mc_rtc/log/FlatLog.h>

/** Helper functions to work with FlatLog */

namespace utils
{

inline std::map<std::string, mc_rtc::log::LogType> entries(const mc_rtc::log::FlatLog & log,
                                                           const std::vector<std::string> & entriesFilter)
{
  std::map<std::string, mc_rtc::log::LogType> ret;
  for(const auto & e : entriesFilter)
  {
    if(!log.has(e))
    {
      mc_rtc::log::warning("Requested log entry named {} but this entry is not part of the log, ignoring", e);
    }
  }
  for(const auto & e : log.entries())
  {
    if(entriesFilter.size() && std::find(entriesFilter.begin(), entriesFilter.end(), e) == entriesFilter.end())
    {
      continue;
    }
    auto t = log.type(e);
    if(t != mc_rtc::log::LogType::None)
    {
      ret[e] = t;
    }
    else
    {
      mc_rtc::log::warning("{} cannot be converted into a flat log", e);
    }
  }
  return ret;
}

inline size_t VectorXdEntrySize(const mc_rtc::log::FlatLog & log, const std::string & entry)
{
  size_t s = 0;
  auto data = log.getRaw<Eigen::VectorXd>(entry);
  for(const auto & v : data)
  {
    if(v)
    {
      s = std::max<size_t>(s, static_cast<size_t>(v->size()));
    }
  }
  return s;
}

inline size_t VectorEntrySize(const mc_rtc::log::FlatLog & log, const std::string & entry)
{
  size_t s = 0;
  auto data = log.getRaw<std::vector<double>>(entry);
  for(const auto & v : data)
  {
    if(v)
    {
      s = std::max<size_t>(s, v->size());
    }
  }
  return s;
}

inline size_t entrySize(const mc_rtc::log::FlatLog & log, const std::string & entry, const mc_rtc::log::LogType & t)
{
  switch(t)
  {
    case mc_rtc::log::LogType::Bool:
    case mc_rtc::log::LogType::Int8_t:
    case mc_rtc::log::LogType::Int16_t:
    case mc_rtc::log::LogType::Int32_t:
    case mc_rtc::log::LogType::Int64_t:
    case mc_rtc::log::LogType::Uint8_t:
    case mc_rtc::log::LogType::Uint16_t:
    case mc_rtc::log::LogType::Uint32_t:
    case mc_rtc::log::LogType::Uint64_t:
    case mc_rtc::log::LogType::Float:
    case mc_rtc::log::LogType::Double:
    case mc_rtc::log::LogType::String:
      return 1;
    case mc_rtc::log::LogType::Quaterniond:
      return 4;
    case mc_rtc::log::LogType::Vector2d:
      return 2;
    case mc_rtc::log::LogType::Vector3d:
      return 3;
    case mc_rtc::log::LogType::Vector6d:
      return 6;
    case mc_rtc::log::LogType::VectorXd:
      return VectorXdEntrySize(log, entry);
    case mc_rtc::log::LogType::PTransformd:
      return 7;
    case mc_rtc::log::LogType::ForceVecd:
    case mc_rtc::log::LogType::MotionVecd:
      return 6;
    case mc_rtc::log::LogType::VectorDouble:
      return VectorEntrySize(log, entry);
    case mc_rtc::log::LogType::None:
    default:
      return 0;
  }
}

} // namespace utils
