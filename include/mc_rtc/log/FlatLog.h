/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/log/utils.h>
#include <mc_rtc/utils_api.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <stdexcept>
#include <string>
#include <unordered_map>

namespace mc_rtc
{

namespace log
{

/** Holds type information about a record */
struct MC_RTC_UTILS_DLLAPI RecordType
{
  /** Type of the record, None if no data was recorded */
  LogType type = LogType::None;
  /** Holds sub-types for an std::vector entry */
  std::vector<LogType> vectorType = {};
};

inline bool operator<(const RecordType & lhs, const RecordType & rhs)
{
  return lhs.type < rhs.type || (lhs.type == rhs.type && lhs.vectorType < rhs.vectorType);
}

/** From an on-disk binary log recorded by mc_rtc, return a flat structure */
struct MC_RTC_UTILS_DLLAPI FlatLog
{
  /** Default constructor, empty log */
  FlatLog() = default;

  /** Load a file into the log */
  FlatLog(const std::string & fpath);

  /** Delete copy constructor */
  FlatLog(const FlatLog &) = delete;

  /** Delete copy assignment */
  FlatLog & operator=(const FlatLog &) = delete;

  /** Load a file into the log, erase the current content of the flat log */
  void load(const std::string & fpath);

  /** Append a file into the flat log, the resulting content is the concatenation of the two logs*/
  void append(const std::string & fpath);

  /** Returns the size of the log */
  size_t size() const;

  /** Returns a sorted list of entries in the log */
  std::set<std::string> entries() const;

  /** Returns true if the log has the provided entry */
  bool has(const std::string & entry) const;

  /** Returns available types for an entry */
  std::set<RecordType> types(const std::string & entry) const;

  /** Get the first non None type for an entry */
  RecordType type(const std::string & entry) const;

  /** Get a type record entry.
   *
   * Get null pointer entry when the record data type does not match the
   * requested data type
   *
   * \param entry Entry to get
   *
   */
  template<typename T>
  std::vector<const T *> getRaw(const std::string & entry) const;

  /** Get a typed record entry
   *
   * Get a default value when the record data type does not match the requested
   * data type
   *
   * \param entry Entry to get
   *
   * \param def Default value when the record data does not match requested
   * data
   *
   */
  template<typename T>
  std::vector<T> get(const std::string & entry, const T & def) const;

  /** Get a typed record entry
   *
   * When the record data type does not match the requested data type, the
   * vector is automatically filled up with the last (or first if relevant)
   * recorded data
   *
   * \param entry Entry to get
   *
   */
  template<typename T>
  std::vector<T> get(const std::string & entry) const;

  /** Get a typed record entry at a given index
   *
   * When the record datat type does not match the requested data type, returns the default value provided.
   *
   * \param entry Entry to get
   *
   * \param i Index to get
   *
   * \param def Default value when the record data does not match requested
   * data
   *
   */
  template<typename T>
  T get(const std::string & entry, size_t i, const T & def) const;

  struct record
  {
    using unique_void_ptr = std::unique_ptr<void, void (*)(void const *)>;
    record();
    record(RecordType t, unique_void_ptr && d) : type(t), data(std::move(d)) {}
    record(const record &) = delete;
    record & operator=(const record &) = delete;
    record(record &&) = default;
    record & operator=(record &&) = default;
    RecordType type = {};
    unique_void_ptr data;
  };
  struct entry
  {
    std::string name;
    std::vector<record> records;
  };

private:
  std::vector<entry> data_;

  /** Retrieve records for a given entry */
  const std::vector<record> & at(const std::string & entry) const;

  /** Retrieve the index of a given entry, creates the entry if it doesn't exist */
  size_t index(const std::string & entry, size_t size);
};

} // namespace log

} // namespace mc_rtc

#include "FlatLog.hpp"
