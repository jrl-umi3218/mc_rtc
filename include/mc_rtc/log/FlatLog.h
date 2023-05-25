/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/log/Logger.h>
#include <mc_rtc/log/utils.h>
#include <mc_rtc/utils_api.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <stdexcept>
#include <string>
#include <unordered_map>

namespace mc_rtc::log
{

namespace details
{

template<typename T>
struct GetRawReturnType
{
  using type = T;
};

template<int N, int _Options, int _MaxRows, int _MaxCols>
struct GetRawReturnType<Eigen::Matrix<double, N, 1, _Options, _MaxRows, _MaxCols>>
{
  using type = std::conditional_t<N == 2 || N == 3 || N == 6,
                                  Eigen::Matrix<double, N, 1, _Options, _MaxRows, _MaxCols>,
                                  Eigen::VectorXd>;
};

} // namespace details

/** From an on-disk binary log recorded by mc_rtc, return a flat structure */
struct MC_RTC_UTILS_DLLAPI FlatLog
{
  template<typename T>
  using get_raw_return_t = typename details::GetRawReturnType<T>::type;

  /** Default constructor, empty log */
  FlatLog() = default;

  /** Load a file into the log */
  FlatLog(const std::string & fpath);

  FlatLog(const FlatLog &) = delete;
  FlatLog & operator=(const FlatLog &) = delete;

  FlatLog(FlatLog &&) = default;
  FlatLog & operator=(FlatLog &&) = default;

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
  std::set<LogType> types(const std::string & entry) const;

  /** Get the first non None type for an entry */
  LogType type(const std::string & entry) const;

  /** Get the type at index i */
  LogType type(const std::string & entry, size_t i) const;

  /** Get a type record entry.
   *
   * Get null pointer entry when the record data type does not match the
   * requested data type
   *
   * \param entry Entry to get
   *
   */
  template<typename T>
  std::vector<const get_raw_return_t<T> *> getRaw(const std::string & entry) const;

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
  std::vector<get_raw_return_t<T>> get(const std::string & entry, const T & def) const;

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
  std::vector<get_raw_return_t<T>> get(const std::string & entry) const;

  /** Get a typed record entry at a given index
   *
   * When the record type does not match the requested data type, returns the default value provided.
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
  get_raw_return_t<T> get(const std::string & entry, size_t i, const T & def) const;

  /** Get a typed raw entry at a given index
   *
   * Returns nullptr when the requested type does not match the record type
   *
   * \param entry Entry to get
   *
   * \param i Index to get
   *
   */
  template<typename T>
  const get_raw_return_t<T> * getRaw(const std::string & entry, size_t i) const;

  /** Returns all the GUI events that happened during the log recording */
  inline const std::vector<std::vector<Logger::GUIEvent>> & guiEvents() const noexcept { return gui_events_; }

  /** Returns the meta information contained in the log
   *
   * nullopt when the file does not contain such information
   */
  inline const std::optional<Logger::Meta> & meta() const noexcept { return meta_; }

  struct record
  {
    using unique_void_ptr = std::unique_ptr<void, void (*)(void const *)>;
    record();
    record(LogType t, unique_void_ptr && d) : type(t), data(std::move(d)) {}
    record(const record &) = delete;
    record & operator=(const record &) = delete;
    record(record &&) = default;
    record & operator=(record &&) = default;
    LogType type = mc_rtc::log::LogType::None;
    unique_void_ptr data;
  };
  struct entry
  {
    std::string name;
    std::vector<record> records;
  };

private:
  std::vector<entry> data_;
  std::vector<std::vector<Logger::GUIEvent>> gui_events_;
  std::optional<Logger::Meta> meta_;

  /** Retrieve records for a given entry */
  const std::vector<record> & at(const std::string & entry) const;

  /** Retrieve the index of a given entry, creates the entry if it doesn't exist */
  size_t index(const std::string & entry, size_t size);

  /** Append a flat file to the log, all entries will be either double or strings */
  void appendFlat(const std::string & fpath);

  /** Append a binary file to the log */
  void appendBin(const std::string & fpath);
};

} // namespace mc_rtc::log

#include "FlatLog.hpp"
