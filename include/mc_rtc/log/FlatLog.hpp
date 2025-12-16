/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/logging.h>

#include "FlatLog.h"

namespace mc_rtc::log
{

namespace details
{

template<typename T>
struct CheckLogType
{
  static bool check(const LogType & t) { return t == mc_rtc::log::GetLogType<T>::type; }
};

template<typename T>
struct CastLogRecord
{
  static const T * cast(const FlatLog::record & r) { return static_cast<const T *>(r.data.get()); }
};

template<int N, int _Options, int _MaxRows, int _MaxCols>
struct CastLogRecord<Eigen::Matrix<double, N, 1, _Options, _MaxRows, _MaxCols>>
{
  static const FlatLog::get_raw_return_t<Eigen::Matrix<double, N, 1, _Options, _MaxRows, _MaxCols>> * cast(
      const FlatLog::record & r)
  {
    if constexpr(N == -1 || N == 2 || N == 3 || N == 6)
    {
      return static_cast<const Eigen::Matrix<double, N, 1, _Options, _MaxRows, _MaxCols> *>(r.data.get());
    }
    else
    {
      auto v = static_cast<const Eigen::VectorXd *>(r.data.get());
      if(v->size() == N) { return v; }
      else
      {
        return nullptr;
      }
    }
  }
};

template<typename T>
const FlatLog::get_raw_return_t<T> * record_cast(const FlatLog::record & r)
{
  if(CheckLogType<T>::check(r.type)) { return CastLogRecord<T>::cast(r); }
  return nullptr;
}

template<typename T>
bool convert(const FlatLog::record & r, T & out)
{
  auto ptr = record_cast<T>(r);
  if(ptr) { out = *ptr; }
  return ptr;
}

} // namespace details

template<typename T>
auto FlatLog::getRaw(const std::string & entry) const -> std::vector<const get_raw_return_t<T> *>
{
  if(!has(entry))
  {
    log::error("No entry named {} in the loaded log", entry);
    return {};
  }
  std::vector<const T *> ret;
  const auto & data = at(entry);
  ret.reserve(data.size());
  for(const auto & r : data) { ret.push_back(details::record_cast<T>(r)); }
  return ret;
}

template<typename T>
auto FlatLog::get(const std::string & entry, const T & def) const -> std::vector<get_raw_return_t<T>>
{
  if(!has(entry))
  {
    log::error("No entry named {} in the loaded log", entry);
    return {};
  }
  const auto & data = at(entry);
  std::vector<T> ret(data.size(), def);
  for(size_t i = 0; i < data.size(); ++i) { details::convert<T>(data[i], ret[i]); }
  return ret;
}

template<>
inline std::vector<bool> FlatLog::get(const std::string & entry, const bool & def) const
{
  if(!has(entry))
  {
    log::error("No entry named {} in the loaded log", entry);
    return {};
  }
  const auto & data = at(entry);
  std::vector<bool> ret(data.size(), def);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const bool * ptr = details::record_cast<bool>(data[i]);
    if(ptr) { ret[i] = *ptr; }
  }
  return ret;
}

template<typename T>
auto FlatLog::get(const std::string & entry) const -> std::vector<get_raw_return_t<T>>
{
  if(!has(entry))
  {
    log::error("No entry named {} in the loaded log", entry);
    return {};
  }
  const auto & data = at(entry);
  std::vector<T> ret;
  size_t start_i = 0;
  T def{};
  while(start_i < data.size())
  {
    const auto & r = data[start_i];
    if(details::convert<T>(r, def)) { break; }
    start_i++;
  }
  if(start_i == data.size())
  {
    log::error("{} was not logged as the requested data type", entry);
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
auto FlatLog::get(const std::string & entry, size_t i, const T & def) const -> get_raw_return_t<T>
{
  const T * data = getRaw<T>(entry, i);
  if(data) { return *data; }
  return def;
}

template<typename T>
auto FlatLog::getRaw(const std::string & entry, size_t i) const -> const get_raw_return_t<T> *
{
  if(!has(entry))
  {
    log::error("No entry named {} in the loaded log", entry);
    return nullptr;
  }
  const auto & data = at(entry);
  if(i >= data.size())
  {
    log::error("Requested data ({}) out of available range ({}, available: {})", entry, i, data.size());
    return nullptr;
  }
  return details::record_cast<T>(data[i]);
}

} // namespace mc_rtc::log
