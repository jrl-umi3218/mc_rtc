#include <mc_rtc/log/Logger.h>
#include <mc_rtc/log/iterate_binary_log.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include "internals/LogEntry.h"
#include <fstream>

namespace mc_rtc::log
{

bool iterate_binary_log(const std::string & f,
                        const iterate_binary_log_callback & callback,
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
  if(memcmp(buffer.data(), &mc_rtc::Logger::magic, sizeof(mc_rtc::Logger::magic) - 1) != 0)
  {
    log::error("Log {} is not a valid mc_rtc binary log (Invalid magic number)", f);
    return false;
  }
  int8_t version = static_cast<int8_t>(buffer.data()[sizeof(mc_rtc::Logger::magic) - 1] - mc_rtc::Logger::magic[3]);
  if(version < 0)
  {
    log::error("Log {} is not a valid mc_rtc binary log (Invalid version number)", f);
    return false;
  }
  if(version > mc_rtc::Logger::version)
  {
    log::error("Log {} cannot be read by this version of mc_rtc ({} > {})", version, mc_rtc::Logger::version);
    return false;
  }
  size_t t_index = 0;
  bool extract_t = time.size() != 0;

  std::vector<internal::TypedKey> keys;
  std::optional<Logger::Meta> meta;

  while(ifs)
  {
    uint64_t entrySize = 0;
    ifs.read((char *)&entrySize, sizeof(uint64_t));
    if(!ifs) { break; }
    while(buffer.size() < entrySize) { buffer.resize(2 * buffer.size()); }
    ifs.read(buffer.data(), static_cast<int>(entrySize));
    if(!ifs) { break; }
    bool keys_changed = false;
    std::vector<Logger::GUIEvent> events;
    internal::LogEntry log(version, buffer, entrySize, meta, keys, events, keys_changed, extract);
    if(!log.valid()) { return false; }
    if(extract_t)
    {
      auto t_it = std::find_if(keys.begin(), keys.end(), [&](const auto & k) { return k.key == time; });
      if(t_it == keys.end())
      {
        log::error("Request time key: {} not found in log", time);
        return false;
      }
      if(t_it->type != LogType::Double)
      {
        log::error("Time key: {} not recording double", time);
        return false;
      }
      t_index = static_cast<size_t>(std::distance(keys.begin(), t_it));
    }
    std::optional<double> t;
    if(extract_t) { t = log.getTime(t_index); }
    auto keys_str = [&]()
    {
      std::vector<std::string> keys_str;
      if(keys_changed)
      {
        keys_str.reserve(keys.size());
        for(const auto & k : keys) { keys_str.push_back(k.key); }
      }
      return keys_str;
    }();
    if(!callback(
           IterateBinaryLogData{keys_str, log.records(), events, t,
                                [&log](mc_rtc::MessagePackBuilder & builder, const std::vector<std::string> & keys)
                                { log.copy(builder, keys); }, buffer.data(), entrySize, meta}))
    {
      return false;
    }
  }
  return true;
}

} // namespace mc_rtc::log
