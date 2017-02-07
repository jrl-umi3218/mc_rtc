#pragma once

#include <mc_control/mc_controller.h>

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
  void log_header(const std::string & ctl_name, MCController * controller);

  /*! \brief Log controller's data
   *
   * Print controller data to the log.
   *
   * \param controller The running controller
   */
  void log_data(MCGlobalController & gc, MCController * controller);
private:
  std::unique_ptr<LoggerImpl> impl;
};

}
