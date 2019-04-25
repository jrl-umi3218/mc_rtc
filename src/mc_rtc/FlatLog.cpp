/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/log/FlatLog.h>

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
  std::vector<size_t> currentIndexes = {};
  std::vector<size_t> missingIndexes = {};
  size_t size = 0;
  if(data_.size())
  {
    size = data_[0].records.size();
  }
  std::vector<char> buffer(1024);
  while(ifs)
  {
    size_t entrySize = 0;
    ifs.read((char *)&entrySize, sizeof(size_t));
    if(!ifs)
    {
      break;
    }
    while(buffer.size() < entrySize)
    {
      buffer.resize(2 * buffer.size());
    }
    ifs.read(buffer.data(), entrySize);
    if(!ifs)
    {
      break;
    }
    internal::LogEntry log(buffer, entrySize);
    if(!log.valid())
    {
      return;
    }
    if(log.keys().size())
    {
      for(const auto & k : missingIndexes)
      {
        data_[k].records.resize(size);
      }
      currentIndexes.clear();
      for(const auto & k : log.keys())
      {
        currentIndexes.push_back(index(k, size));
      }
      missingIndexes.clear();
      {
        size_t i = 0;
        for(size_t j = 0; j < currentIndexes.size(); ++j)
        {
          while(i < currentIndexes[j])
          {
            missingIndexes.push_back(i);
            i++;
          }
        }
        for(size_t i = currentIndexes.back(); i < data_.size(); ++i)
        {
          missingIndexes.push_back(i);
        }
      }
    }
    auto & recordsIn = log.records();
    for(size_t i = 0; i < recordsIn.size(); ++i)
    {
      auto & records = data_[currentIndexes[i]].records;
      records.push_back(std::move(recordsIn[i]));
    }
    size += 1;
  }
  for(const auto & k : missingIndexes)
  {
    data_[k].records.resize(size);
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

std::set<RecordType> FlatLog::types(const std::string & entry) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return {};
  }
  std::set<RecordType> ret;
  for(const auto & r : at(entry))
  {
    ret.insert(r.type);
  }
  ret.erase(RecordType{});
  return ret;
}

RecordType FlatLog::type(const std::string & entry) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return {};
  }
  for(const auto & r : at(entry))
  {
    if(r.type.type != mc_rtc::log::LogType::None)
    {
      return r.type;
    }
  }
  return {};
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
