#pragma once

/*
 * Copyright 2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#include <mc_rtc/MessagePackBuilder.h>
#include <mc_rtc/log/FlatLog.h>

namespace mc_rtc
{

namespace log
{

using binary_log_callback =
    std::function<bool(const std::vector<std::string> &, std::vector<FlatLog::record> &, double)>;

using copy_callback = std::function<void(mc_rtc::MessagePackBuilder &, const std::vector<std::string> &)>;

using binary_log_copy_callback = std::function<bool(const std::vector<std::string> &,
                                                    std::vector<FlatLog::record> &,
                                                    double,
                                                    const copy_callback &,
                                                    const char *,
                                                    size_t)>;

/** Iterate over a given binary log data
 *
 * For each entry in the log, this will call the provided callback. The
 * callback has six arguments:
 *
 * - the first are the keys of the entry, note that this is empty if there has
 *   been no change since the previous entry
 *
 * - the second is the data corresponding to the keys. The index of a key in
 *   the keys vector give the index for the corresponding data. This index may
 *   change when keys are updated
 *
 * - the third is the value for the time entry
 *
 * - the fourth is a copy callback that can be used to obtain a copy of the
 *   data with provided keys
 *
 * - the fifth is a pointer to the raw buffer data
 *
 * - the sixth is the size of this buffer
 *
 * If the callback returns false, parsing is aborted
 *
 * \param fpath Path to the binary log file
 *
 * \param callback Called for every entry in the log
 *
 * \param extract If true, data is available as their original C++ type, if
 * false, no data is available. Skipping extraction is much faster
 *
 * \param time Key used as time source. If that key doesn't exist or if it's
 * not a Double record, the parsing will fail. If this parameter is empty, no
 * extraction is attempted and the third callback parameter is always -1
 *
 * \returns True if parsing was sucessful, false otherwise
 */
bool MC_RTC_UTILS_DLLAPI iterate_binary_log(const std::string & fpath,
                                            const binary_log_copy_callback & callback,
                                            bool extract,
                                            const std::string & time = "t");

/** Iterate over a given binary log data
 *
 * This function has the same functionality as its overload but the callback
 * doesn't have to bother with the copy callback.
 */
bool MC_RTC_UTILS_DLLAPI iterate_binary_log(const std::string & fpath,
                                            const binary_log_callback & callback,
                                            bool extract,
                                            const std::string & time = "t");

} // namespace log

} // namespace mc_rtc
