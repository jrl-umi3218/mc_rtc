#pragma once

#include <mc_rtc/log/serialization/MCLog_generated.h>
#include <mc_rtc/utils_api.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <string>
#include <unordered_map>

namespace mc_rtc
{

namespace log
{

/** From an on-disk binary log recorded by mc_rtc, return a flat structure */
struct MC_RTC_UTILS_DLLAPI FlatLog
{
  /** Default constructor, empty log */
  FlatLog() = default;

  /** Load a file into the log */
  FlatLog(const std::string & fpath);

  /** Load a file into the log, erase the current content of the flat log */
  void load(const std::string & fpath);

  /** Append a file into the flat log, the resulting content is the concatenation of the two logs*/
  void append(const std::string & fpath);

  /** Returns a sorted list of entries in the log */
  std::set<std::string> entries() const;

  /** Returns true if the log has the provided entry */
  bool has(const std::string & entry) const;

  /** Get a typed record entry, the vector has the full size of the log with
   * null pointer when data was not recorded at a given point in the log */
  template<typename T>
  std::vector<T const *> get(const std::string & entry) const;

  struct record
  {
    using unique_void_ptr = std::unique_ptr<void, void (*)(void const *)>;
    record();
    record(LogData t, unique_void_ptr && d) : type(t), data(std::move(d)) {}
    LogData type = LogData_NONE;
    unique_void_ptr data;
  };

private:
  std::unordered_map<std::string, std::vector<record>> data_;
};

} // namespace log

} // namespace mc_rtc

#include "FlatLog.hpp"
