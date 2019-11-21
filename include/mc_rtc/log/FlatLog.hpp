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
struct CheckLogType
{
  static bool check(const LogType & t)
  {
    return t == mc_rtc::log::GetLogType<T>::type;
  }
};

template<typename T>
const T * record_cast(const FlatLog::record & r)
{
  if(CheckLogType<T>::check(r.type))
  {
    return static_cast<const T *>(r.data.get());
  }
  return nullptr;
}

template<typename T>
bool convert(const FlatLog::record & r, T & out)
{
  const T * ptr = record_cast<T>(r);
  if(ptr)
  {
    out = *ptr;
  }
  return ptr;
}

} // namespace details

template<typename T>
std::vector<const T *> FlatLog::getRaw(const std::string & entry) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return {};
  }
  std::vector<const T *> ret;
  const auto & data = at(entry);
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
  const auto & data = at(entry);
  std::vector<T> ret(data.size(), def);
  for(size_t i = 0; i < data.size(); ++i)
  {
    details::convert<T>(data[i], ret[i]);
  }
  return ret;
}

template<>
inline std::vector<bool> FlatLog::get(const std::string & entry, const bool & def) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return {};
  }
  const auto & data = at(entry);
  std::vector<bool> ret(data.size(), def);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const bool * ptr = details::record_cast<bool>(data[i]);
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
  const T * data = getRaw<T>(entry, i);
  if(data)
  {
    return *data;
  }
  return def;
}

template<typename T>
const T * FlatLog::getRaw(const std::string & entry, size_t i) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return nullptr;
  }
  const auto & data = at(entry);
  if(i >= data.size())
  {
    LOG_ERROR("Requested data (" << entry << ") out of available range (" << i << ", available: " << data.size() << ")")
    return nullptr;
  }
  return details::record_cast<T>(data[i]);
}

} // namespace log

} // namespace mc_rtc
