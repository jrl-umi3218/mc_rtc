#pragma once

#include <mc_rtc/log/serialization/fb_utils.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/utils_api.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <map>
#include <memory>

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
  /** A function that fills LogData vectors */
  typedef std::function<void(flatbuffers::FlatBufferBuilder&,
                             std::vector<uint8_t>&,
                             std::vector<flatbuffers::Offset<void>>&)> serialize_fn;
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
public:
  /*! \brief Constructor
   *
   * \param policy The chosen logging policy
   *
   * \param directory Path to the directory where log files will be stored
   *
   * \param tmpl Log file template
   */
  Logger(const Policy & policy, const bfs::path & directory, const std::string & tmpl);

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
  void setup(const Policy & policy, const bfs::path & directory, const std::string & tmpl);

  /*! \brief Start logging
   *
   * Print the file header to the log. This should be called at
   * initialization or when a controller switch occurs
   *
   * \param ctl_name Name of the running controller
   *
   * \param timestep Time increment for the log time entry
   */
  void start(const std::string & ctl_name, double timestep);

  /*! \brief Log controller's data
   *
   * Print controller data to the log.
   *
   */
  void log();

  /** Add a log entry into the log
   *
   * This function only accepts callable objects that returns a l/rvalue to a
   * serializable object.
   *
   * \param name Name of the log entry, this should be unique at any given time
   * but the same key can be re-used during the logger's life
   *
   * \param get_fn A function that provides data that should be logged
   *
   */
  template<typename T>
  void addLogEntry(const std::string & name,
                   T get_fn,
                   typename std::enable_if<mc_rtc::log::callback_is_serializable<T>::value>::type * = 0)
  {
    using ret_t = decltype(get_fn());
    using base_t =  typename std::decay<ret_t>::type;
    if(log_entries_.count(name))
    {
      LOG_ERROR("Already logging an entry named " << name)
      return;
    }
    log_entries_changed_ = true;
    log_entries_[name] = [this, get_fn](flatbuffers::FlatBufferBuilder & builder,
                                        std::vector<uint8_t> & types,
                                        std::vector<flatbuffers::Offset<void>> & values)
    {
      mc_rtc::log::AddLogData<base_t>(builder, types, values, get_fn());
    };
  }

  /** Add log entry into the log
   *
   * This function only accepts callable objects that returns a const
   * reference to a vector holding serializable objects.
   *
   * \param name Name of the log entry, this should be unique at any given
   * time but the same key can be re-used during the logger's life
   *
   * \param get_fn A function that provides data that should be logged
   *
   */
  template<typename T>
  void addLogEntry(const std::string & name,
                   T get_fn,
                   typename std::enable_if<mc_rtc::log::callback_is_crv_of_serializable<T>::value>::type * = 0)
  {
    using ret_t = decltype(get_fn());
    using base_t = typename std::decay<ret_t>::type;
    using value_t = typename base_t::value_type;
    if(log_entries_.count(name))
    {
      LOG_ERROR("Already logging an entry named " << name)
      return;
    }
    log_entries_changed_ = true;
    log_vector_entries_size_[name] = 0;
    log_entries_[name] = [this, name, get_fn](flatbuffers::FlatBufferBuilder & builder,
                                        std::vector<uint8_t> & types,
                                        std::vector<flatbuffers::Offset<void>> & values)
    {
      const std::vector<value_t> & v = get_fn();
      if(v.size() != this->log_vector_entries_size_[name])
      {
        log_entries_changed_ = true;
        this->log_vector_entries_size_[name] = v.size();
      }
      for(const auto & e : v)
      {
        mc_rtc::log::AddLogData<value_t>(builder, types, values, e);
      }
    };
  }

  /** Remove a log entry from the log
   *
   * This has no effect if the log entry does not exist.
   *
   * \param name Name of the entry
   *
   */
  void removeLogEntry(const std::string & name);
private:
  /** Store implementation detail related to the logging policy */
  std::shared_ptr<LoggerImpl> impl_ = nullptr;
  /** Set to true when log entries are added or removed */
  bool log_entries_changed_ = false;
  /** Contains all the log entries callback */
  std::map<std::string, serialize_fn> log_entries_ = {};
  /** For vector entries, retain the size of the vector in the previous call */
  std::map<std::string, size_t> log_vector_entries_size_ = {};
};

}
