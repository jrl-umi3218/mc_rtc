#pragma once

namespace mc_control
{
  struct Logger;
}

#include <mc_control/mc_controller.h>
#include <mc_control/log/serialization/fb_utils.h>

#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <memory>

namespace mc_control
{

struct LoggerImpl;
/*! \brief Logs controller data to disk
 *
 * The user can select a desired logging policy that will impact the behaviour
 * of this class.
 *
 * See mc_control::Logger::Policy documentation for details on available
 * policies
 */
struct Logger
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

  /*! \brief Start logging
   *
   * Print the file header to the log. This should be called at
   * initialization or when a controller switch occurs
   *
   * \param ctl_name Name of the running controller
   *
   * \param controller The running controller
   */
  void start(const std::string & ctl_name, MCController * controller);

  /*! \brief Log controller's data
   *
   * Print controller data to the log.
   *
   * \param controller The running controller
   */
  void log();

  /** Add a log entry into the log
   *
   * \param name Name of the log entry, this should be unique at any given time but the same key can be re-used during the logger's life
   *
   * \param get_fn A function that provides data that should be logged
   *
   */
  template<typename T>
  void addLogEntry(const std::string & name,
                   T get_fn)
  {
    typedef decltype(get_fn()) ret_t;
    typedef typename std::decay<ret_t>::type base_t;
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
      mc_control::log::AddLogData<base_t>(builder, types, values, get_fn());
    };
  }

  /** Remove a log entry from the log */
  void removeLogEntry(const std::string & name);
private:
  std::shared_ptr<LoggerImpl> impl_ = nullptr;
  bool log_entries_changed_ = false;
  std::map<std::string, serialize_fn> log_entries_ = {};
};

}
