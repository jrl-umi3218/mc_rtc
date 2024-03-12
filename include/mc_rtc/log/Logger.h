/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/log/utils.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/utils_api.h>

#include <mc_rtc/Configuration.h>

#include <memory>
#include <unordered_map>
#include <variant>

namespace mc_rtc
{

struct LoggerImpl;
/*! \brief Logs controller data to disk
 *
 * The user can select a desired logging policy that will impact the behaviour
 * of this class.
 *
 * See mc_rtc::Logger::Policy documentation for details on available
 * policies
 */
struct MC_RTC_UTILS_DLLAPI Logger
{
public:
  /** Magic number used to identify binary logs */
  static const uint8_t magic[4];
  /** Version of the log format
   *
   * This is stored in the binary file as data[3] - magic[3]
   */
  static const uint8_t version;
  /** A function that fills LogData vectors */
  typedef std::function<void(mc_rtc::MessagePackBuilder &)> serialize_fn;
  /*! \brief Defines available policies for the logger */
  enum struct Policy
  {
    /*! \brief Non-threaded policy
     *
     * Using this policy, the logging disk operations are done in the same
     * thread as the global controller running thread, this is well suited
     * for simulations and environments where you can guarantee a very fast
     * access to the disk (e.g. logging to a ramdisk)
     */
    NON_THREADED = 0,
    /*! \brief Threaded policy
     *
     * Using this policy, the logging disk operations are done in a separate
     * thread from the global controller running thread. As a result, some
     * buffering occurs and you might lose some data if the controller
     * crashes. This is intended for real-time environments.
     */
    THREADED = 1
  };

  /*! \brief Data for a key added event */
  struct KeyAddedEvent
  {
    /** Type of the key being added */
    log::LogType type;
    /** Name of the key being added */
    std::string key;
  };

  /*! \brief Data for a key removed event */
  struct KeyRemovedEvent
  {
    /** Name of the key being removed */
    std::string key;
  };

  /*! \brief GUI callback event */
  struct GUIEvent
  {
    /** Category of the item */
    std::vector<std::string> category;
    /** Name of the time */
    std::string name;
    /** Payload of the callback */
    mc_rtc::Configuration data;
  };

  /*! \brief Start event
   *
   * By construction this should appear only once in the log
   *
   * In the current implementation this writes \ref Meta into the log
   */
  struct StartEvent
  {
  };

  using LogEvent = std::variant<KeyAddedEvent, KeyRemovedEvent, GUIEvent, StartEvent>;

  /*! \brief Log meta data written in the first call to \ref log after a call to \start */
  struct Meta
  {
    /** Timestep of the log */
    double timestep;
    /** Name of the main robot this reports */
    std::string main_robot;
    /** Module parameters for the main robot */
    std::vector<std::string> main_robot_module;
    /** Initial position of robots in the world */
    std::map<std::string, sva::PTransformd> init;
    /** Initial configuration of robots in the world */
    std::map<std::string, std::vector<std::vector<double>>> init_q;
    /** Calibration used for the force sensors */
    std::map<std::string, std::map<std::string, mc_rtc::Configuration>> calibs;
  };

public:
  /*! \brief Constructor
   *
   * \param policy The chosen logging policy
   *
   * \param directory Path to the directory where log files will be stored
   *
   * \param tmpl Log file template
   */
  Logger(const Policy & policy, const std::string & directory, const std::string & tmpl);

  /*! \brief Destructor */
  ~Logger();

  /*! \brief Setup the constructor configuration
   *
   * \param policy The chosen logging policy
   *
   * \param directory Path to the directory where log files will be stored
   *
   * \param tmpl Log file template
   */
  void setup(const Policy & policy, const std::string & directory, const std::string & tmpl);

  /*! \brief Access the log's metadata */
  inline Meta & meta() noexcept { return meta_; }

  /*! \brief Access the log's metadata (const) */
  inline const Meta & meta() const noexcept { return meta_; }

  /*! \brief Start logging
   *
   * Print the file header to the log. This should be called at initialization
   * or when a controller switch occurs
   *
   * \param ctl_name Name of the running controller
   *
   * \param timestep Time increment for the log time entry
   *
   * \param resume If true, start the time entry at the current value,
   * otherwise, start at 0
   *
   * \param start_t Start at the specified time, only used if resume is false
   */
  void start(const std::string & ctl_name, double timestep, bool resume = false, double start_t = 0.0);

  /*! \brief Open a new file for logging
   *
   * This is similar to \ref start but disregard the log directory and template. Instead the logging happens in the
   * provided file: \p file.
   *
   * \param file File where data is going to be written
   *
   * \param timestep Time increment for the log time entry
   *
   * \param start_t Start the log at the specified time
   */
  void open(const std::string & file, double timestep, double start_t = 0.0);

  /*! \brief Log controller's data
   *
   * Print controller data to the log.
   *
   */
  void log();

  /** Add a log entry into the log with the provided source
   *
   * This function only accepts callable objects that returns a l/rvalue to a
   * serializable object.
   *
   * The log entry can be removed by calling:
   * - \ref removeLogEntry with the same name
   * - \ref removeLogEntries with the same source
   *
   * The later remove all other log entries from the same source
   *
   * \param name Name of the log entry, this should be unique at any given time
   * but the same key can be re-used during the logger's life
   *
   * \param source Source of the log entry
   *
   * \param get_fn A function that provides data that should be logged
   *
   * \param overwrite If true, overwrite the previous entry of the same name (if any), otherwise log and return
   *
   */
  template<typename CallbackT,
           typename SourceT = void,
           typename std::enable_if<mc_rtc::log::callback_is_serializable<CallbackT>::value, int>::type = 0>
  void addLogEntry(const std::string & name, const SourceT * source, CallbackT && get_fn, bool overwrite = false)
  {
    using ret_t = decltype(get_fn());
    using base_t = typename std::decay<ret_t>::type;
    auto it = find_entry(name);
    if(it != log_entries_.end())
    {
      if(!overwrite)
      {
        log::error("Already logging an entry named {}", name);
        return;
      }
      else { log_entries_.erase(it); }
    }
    auto log_type = log::callback_is_serializable<CallbackT>::log_type;
    log_events_.push_back(KeyAddedEvent{log_type, name});
    log_entries_.push_back({log_type, name, source, [get_fn](mc_rtc::MessagePackBuilder & builder) mutable
                            { mc_rtc::log::LogWriter<base_t>::write(get_fn(), builder); }});
  }

  /** Add a log entry from a source and a compile-time pointer to member
   *
   * This is slightly more efficient than the source + pointer to member version at the cost of annoying syntax,
   * typically:
   * ```cpp
   * logger.addLogEntry<decltype(&Source::member), &Source::member>("name", &source);
   * ```
   *
   * \param name Name of the log entry
   *
   * \param source Source of the log entry
   *
   * \param overwrite If true, overwrite the previous entry of the same name (if any), otherwise log and return
   *
   */
  template<typename MemberPtrT,
           MemberPtrT member,
           typename SourceT,
           typename std::enable_if<mc_rtc::log::is_serializable_member<MemberPtrT>::value, int>::type = 0>
  void addLogEntry(const std::string & name, const SourceT * source, bool overwrite = false)
  {
    using MemberT = decltype(source->*member);
    addLogEntry(name, source, [source]() -> const MemberT & { return source->*member; }, overwrite);
  }

  /** Add a log entry from a source and a compile-time pointer to method
   *
   * This is slightly more efficient than the source + pointer to method version at the cost of annoying syntax,
   * typically:
   * ```cpp
   * logger.addLogEntry<decltype(&Source::method), &Source::method>("name", &source);
   * ```
   *
   * \param name Name of the log entry
   *
   * \param source Source of the log entry
   *
   * \param overwrite If true, overwrite the previous entry of the same name (if any), otherwise log and return
   *
   */
  template<typename MethodPtrT,
           MethodPtrT method,
           typename SourceT,
           typename std::enable_if<mc_rtc::log::is_serializable_getter<MethodPtrT>::value, int>::type = 0>
  void addLogEntry(const std::string & name, const SourceT * source, bool overwrite = false)
  {
    using MethodRetT = decltype((source->*method)());
    addLogEntry(name, source, [source]() -> MethodRetT { return (source->*method)(); }, overwrite);
  }

  /** Add a log entry into the log with no source
   *
   * This function only accepts callable objects that returns a l/rvalue to a
   * serializable object.
   *
   * The log entry can only be removed by calling \ref removeLogEntry with the same name
   *
   * \param name Name of the log entry, this should be unique at any given time
   * but the same key can be re-used during the logger's life
   *
   * \param get_fn A function that provides data that should be logged
   *
   * \param overwrite If true, overwrite the previous entry of the same name (if any), otherwise log and return
   *
   */
  template<typename T, typename std::enable_if<mc_rtc::log::callback_is_serializable<T>::value, int>::type = 0>
  void addLogEntry(const std::string & name, T && get_fn, bool overwrite = false)
  {
    addLogEntry(name, static_cast<const void *>(nullptr), std::forward<T>(get_fn), overwrite);
  }

  /** Add multiple entries at once with the same entry
   *
   * \see addLogEntry for requirements on the callbacks
   *
   * \param source Source of the log entry
   *
   * \param name Name of the log entry
   *
   * \param get_fn A function that provides data that should be logged
   *
   * \param args A serie of name, get_fn values
   */
  template<typename SourceT, typename CallbackT, typename... Args>
  void addLogEntries(const SourceT * source, const std::string & name, CallbackT && get_fn, Args &&... args)
  {
    addLogEntry(name, source, get_fn);
    addLogEntries(source, std::forward<Args>(args)...);
  }

  /** Add a GUI event to the log
   *
   * \param event Event being added to the log
   */
  inline void addGUIEvent(GUIEvent && event) { log_events_.push_back(std::move(event)); }

  /** Remove a log entry from the log
   *
   * This has no effect if the log entry does not exist.
   *
   * \param name Name of the entry
   *
   */
  void removeLogEntry(const std::string & name);

  /** Remove all log entries from a given source
   *
   * This has no effect if no log entry is associated to the given source.
   *
   * \param source Source whose entries should be removed
   *
   */
  void removeLogEntries(const void * source);

  /** Return the time elapsed since the controller start */
  double t() const;

  /** Access the file opened by this Logger
   *
   * \note This is empty before \ref start or \ref open has been called
   */
  const std::string & path() const;

  /** Flush the log data to disk (only implemented in the synchronous method) */
  void flush();

  /** Returns the number of entries currently in the log */
  inline size_t size() const { return log_entries_.size(); }

  /** Remove all entries (except t) from the log
   *
   * \param record If true record the removal events in the log
   *
   * Note: if the \p record parameter is false and the log is written again it might be invalid
   */
  void clear(bool record = true);

private:
  /** Hold information about a log entry stored in this instance */
  struct LogEntry
  {
    /** Type of the entry */
    log::LogType type;
    /** Name of the entry */
    std::string key;
    /** What is the data source (can be nullptr) */
    const void * source;
    /** Callback to log data */
    serialize_fn log_cb;
  };
  /** Store implementation detail related to the logging policy */
  std::shared_ptr<LoggerImpl> impl_ = nullptr;
  /** Meta data for this instance */
  Meta meta_;
  /** Events that happened since the last time we wrote to the log */
  std::vector<LogEvent> log_events_;
  /** Contains all the log entries */
  std::vector<LogEntry> log_entries_;

  std::vector<LogEntry>::iterator find_entry(const std::string & name);

  /** Terminal condition for addLogEntries */
  template<typename SourceT>
  void addLogEntries(const SourceT *)
  {
  }
};

/** Helper to log members or methods with "this" source to the logger variable */
#define MC_RTC_LOG_HELPER(NAME, MEMBER)                                       \
  do {                                                                        \
    using ThisT = typename std::remove_pointer<decltype(this)>::type;         \
    logger.addLogEntry<decltype(&ThisT::MEMBER), &ThisT::MEMBER>(NAME, this); \
  } while(0)

/** Helper to log ambiguous getter methods */
#define MC_RTC_LOG_GETTER(NAME, METHOD)                                          \
  do {                                                                           \
    using MethodRetT = decltype(this->METHOD());                                 \
    logger.addLogEntry(NAME, this, [this]() -> MethodRetT { return METHOD(); }); \
  } while(0)

} // namespace mc_rtc
