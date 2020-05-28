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

bool iterate_binary_log(const std::string & f,
                        const binary_log_copy_callback & callback,
                        bool extract,
                        const std::string & time)
{
  auto fpath = bfs::path(f);
  if(!bfs::exists(f) || !bfs::is_regular(f))
  {
    log::error("Could not open log {}, file does not exist", f);
    return false;
  }
  std::ifstream ifs(f, std::ifstream::binary);
  if(!ifs.is_open())
  {
    log::error("Failed to open {}", f);
    return false;
  }
  std::vector<char> buffer(1024);
  ifs.read(buffer.data(), sizeof(mc_rtc::Logger::magic));
  if(memcmp(buffer.data(), &mc_rtc::Logger::magic, sizeof(mc_rtc::Logger::magic)) != 0)
  {
    log::error("Log {} is not a valid mc_rtc binary log (Invalid magic number)", f);
    return false;
  }
  size_t t_index = 0;
  bool extract_t = time.size() != 0;
  while(ifs)
  {
    uint64_t entrySize = 0;
    ifs.read((char *)&entrySize, sizeof(uint64_t));
    if(!ifs)
    {
      break;
    }
    while(buffer.size() < entrySize)
    {
      buffer.resize(2 * buffer.size());
    }
    ifs.read(buffer.data(), static_cast<int>(entrySize));
    if(!ifs)
    {
      break;
    }
    internal::LogEntry log(buffer, entrySize, extract);
    if(!log.valid())
    {
      return false;
    }
    if(extract_t && log.keys().size())
    {
      t_index = log.keys().size();
      for(size_t i = 0; i < log.keys().size(); ++i)
      {
        const auto & k = log.keys()[i];
        if(k == time)
        {
          t_index = i;
        }
      }
      if(t_index == log.keys().size())
      {
        log::error("Request time key: {} not found in log", time);
        return false;
      }
      if(log.records()[t_index].type != LogType::Double)
      {
        log::error("Time key: {} not recording double", time);
        return false;
      }
    }
    double t = -1;
    if(extract_t)
    {
      t = log.getTime(t_index);
    }
    if(!callback(log.keys(), log.records(), t,
                 [&log](mc_rtc::MessagePackBuilder & builder, const std::vector<std::string> & keys) {
                   log.copy(builder, keys);
                 },
                 buffer.data(), entrySize))
    {
      return false;
    }
  }
  return true;
}

bool iterate_binary_log(const std::string & f,
                        const binary_log_callback & callback,
                        bool extract,
                        const std::string & time)
{
  return iterate_binary_log(
      f,
      binary_log_copy_callback([&callback](const std::vector<std::string> & keys, std::vector<FlatLog::record> & data,
                                           double t, const copy_callback &, const char *,
                                           size_t) mutable { return callback(keys, data, t); }),
      extract, time);
}

} // namespace log

} // namespace mc_rtc
