#pragma once

/*
 * Copyright 2019-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/MessagePackBuilder.h>
#include <mc_rtc/log/FlatLog.h>

namespace mc_rtc::log
{

using copy_callback = std::function<void(mc_rtc::MessagePackBuilder &, const std::vector<std::string> &)>;

/** Raw data provided in every entry of a binary log */
struct MC_RTC_UTILS_DLLAPI IterateBinaryLogData
{
  /** Keys in the log at this iteration */
  const std::vector<std::string> & keys;
  /** Record (type + data) corresponding to the keys (in the same order) */
  std::vector<FlatLog::record> & records;
  /** GUI events that happened at the iteration */
  std::vector<Logger::GUIEvent> & gui_events;
  /** Time value, if extracted */
  std::optional<double> time;
  /** Callback that can be used to copy the entries into a new MsgPack with a different set of keys */
  const copy_callback & copy_cb;
  /** Raw data of the entry */
  const char * raw_data;
  /** Size of the raw data buffer */
  size_t raw_data_size;
  /** Meta data extracted in the log (if any) */
  const std::optional<Logger::Meta> & meta;
};

using iterate_binary_log_callback = std::function<bool(IterateBinaryLogData)>;

using binary_log_callback =
    std::function<bool(const std::vector<std::string> &, std::vector<FlatLog::record> &, double)>;

using binary_log_copy_callback = std::function<bool(const std::vector<std::string> &,
                                                    std::vector<FlatLog::record> &,
                                                    double,
                                                    const copy_callback &,
                                                    const char *,
                                                    uint64_t)>;

/** Iterate over a given binary log data
 *
 * For each entry in the log, this will call the provided callback see \ref IterateBinaryLogData for a documentation of
 * the data available
 *
 * If the callback returns false the parsing is interrupted.
 *
 * \returns True if the parsing was successful, false otherwise
 */
bool MC_RTC_UTILS_DLLAPI iterate_binary_log(const std::string & fpath,
                                            const iterate_binary_log_callback & callback,
                                            bool extract,
                                            const std::string & time = "t");

/** Provided for backward compatibility */
inline bool iterate_binary_log(const std::string & fpath,
                               const binary_log_copy_callback & callback,
                               bool extract,
                               const std::string & time = "t")
{
  return iterate_binary_log(
      fpath,
      [&callback](IterateBinaryLogData data) {
        return callback(data.keys, data.records, data.time.value_or(-1), data.copy_cb, data.raw_data,
                        data.raw_data_size);
      },
      extract, time);
}

/** Provided for backward compatibility */
inline bool iterate_binary_log(const std::string & fpath,
                               const binary_log_callback & callback,
                               bool extract,
                               const std::string & time = "t")
{
  return iterate_binary_log(
      fpath, [&callback](IterateBinaryLogData data)
      { return callback(data.keys, data.records, data.time.value_or(-1)); }, extract, time);
}

} // namespace mc_rtc::log
