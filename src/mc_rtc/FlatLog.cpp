/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/log/Logger.h>
#include <mc_rtc/log/iterate_binary_log.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include "internals/LogEntry.h"
#include <fstream>

namespace mc_rtc
{

namespace log
{

FlatLog::record::record() : type(), data(nullptr, internal::void_deleter<int>) {}

FlatLog::FlatLog(const std::string & fpath)
{
  load(fpath);
}

void FlatLog::load(const std::string & fpath)
{
  data_.clear();
  append(fpath);
}

void FlatLog::append(const std::string & f)
{
  auto fpath = bfs::path(f);
  if(fpath.extension() == ".flat")
  {
    appendFlat(f);
  }
  else
  {
    appendBin(f);
  }
}

void FlatLog::appendBin(const std::string & f)
{
  std::vector<size_t> currentIndexes = {};
  std::vector<size_t> missingIndexes = {};
  size_t size = data_.size() ? data_[0].records.size() : 0;
  mc_rtc::log::binary_log_callback callback = [&](const std::vector<std::string> & ks,
                                                  std::vector<mc_rtc::log::FlatLog::record> & records, double) {
    if(ks.size())
    {
      for(const auto & k : missingIndexes)
      {
        data_[k].records.resize(size);
      }
      currentIndexes.clear();
      for(const auto & k : ks)
      {
        currentIndexes.push_back(index(k, size));
      }
      missingIndexes.clear();
      for(size_t i = 0; i < data_.size(); ++i)
      {
        if(std::find(currentIndexes.begin(), currentIndexes.end(), i) == currentIndexes.end())
        {
          missingIndexes.push_back(i);
        }
      }
    }
    for(size_t i = 0; i < records.size(); ++i)
    {
      auto & out = data_[currentIndexes[i]].records;
      out.push_back(std::move(records[i]));
    }
    size += 1;
    return true;
  };
  iterate_binary_log(f, callback, true, "");
  for(const auto & k : missingIndexes)
  {
    data_[k].records.resize(size);
  }
}

void FlatLog::appendFlat(const std::string & f)
{
  auto fpath = bfs::path(f);
  if(!bfs::exists(f) || !bfs::is_regular(f))
  {
    LOG_ERROR("Could not open log " << f)
    return;
  }
  std::ifstream ifs(f, std::ifstream::binary);
  if(!ifs.is_open())
  {
    LOG_ERROR("Failed to open " << f)
    return;
  }
  size_t size = 0;
  if(data_.size())
  {
    size = data_[0].records.size();
  }
  size_t nEntries = 0;
  ifs.read((char *)&nEntries, sizeof(size_t));
  size_t nsize = 0;
  for(size_t i = 0; i < nEntries; ++i)
  {
    bool is_numeric = false;
    ifs.read((char *)&is_numeric, sizeof(bool));
    size_t sz = 0;
    ifs.read((char *)&sz, sizeof(size_t));
    std::string key(sz, '0');
    ifs.read(&key[0], static_cast<int>(sz * sizeof(char)));
    auto idx = index(key, size);
    ifs.read((char *)&sz, sizeof(size_t));
    auto & entries = data_[idx].records;
    for(size_t i = 0; i < sz; ++i)
    {
      if(is_numeric)
      {
        double data = 0;
        ifs.read((char *)&data, sizeof(double));
        if(std::isnan(data))
        {
          entries.emplace_back(LogType::None, record::unique_void_ptr{nullptr, internal::void_deleter<int>});
        }
        else
        {
          entries.emplace_back(LogType::Double,
                               record::unique_void_ptr{new double(data), internal::void_deleter<double>});
        }
      }
      else
      {
        size_t str_sz = 0;
        ifs.read((char *)&str_sz, sizeof(size_t));
        if(str_sz == 0)
        {
          entries.emplace_back(LogType::None, record::unique_void_ptr{nullptr, internal::void_deleter<int>});
        }
        else
        {
          std::string * str = new std::string(str_sz, '0');
          ifs.read(&((*str)[0]), static_cast<int>(str_sz * sizeof(char)));
          entries.emplace_back(LogType::String, record::unique_void_ptr{str, internal::void_deleter<std::string>});
        }
      }
    }
    nsize = entries.size();
  }
  for(auto & e : data_)
  {
    e.records.resize(nsize);
  }
}

size_t FlatLog::size() const
{
  return data_.size() == 0 ? 0 : data_[0].records.size();
}

std::set<std::string> FlatLog::entries() const
{
  std::set<std::string> ret;
  for(const auto & e : data_)
  {
    ret.insert(e.name);
  }
  return ret;
}

bool FlatLog::has(const std::string & entry) const
{
  return std::find_if(data_.begin(), data_.end(), [&entry](const FlatLog::entry & e) { return e.name == entry; })
         != data_.end();
}

std::set<LogType> FlatLog::types(const std::string & entry) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return {};
  }
  std::set<LogType> ret;
  for(const auto & r : at(entry))
  {
    ret.insert(r.type);
  }
  ret.erase(mc_rtc::log::LogType::None);
  return ret;
}

LogType FlatLog::type(const std::string & entry) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return {};
  }
  for(const auto & r : at(entry))
  {
    if(r.type != mc_rtc::log::LogType::None)
    {
      return r.type;
    }
  }
  return mc_rtc::log::LogType::None;
}

const std::vector<FlatLog::record> & FlatLog::at(const std::string & entry) const
{
  for(const auto & d : data_)
  {
    if(d.name == entry)
    {
      return d.records;
    }
  }
  throw(std::runtime_error("No such entry"));
}

size_t FlatLog::index(const std::string & entry, size_t size)
{
  for(size_t i = 0; i < data_.size(); ++i)
  {
    auto & e = data_[i];
    if(e.name == entry)
    {
      e.records.resize(size);
      return i;
    }
  }
  data_.push_back({entry, std::vector<FlatLog::record>{size}});
  return data_.size() - 1;
}

} // namespace log

} // namespace mc_rtc
