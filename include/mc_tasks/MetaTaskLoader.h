#pragma once

#include <mc_tasks/MetaTask.h>

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

namespace mc_tasks
{

/*! \brief Load a MetaTask
 *
 * Retrieve a MetaTask from a file or an
 * mc_rtc::Configuration object
 *
 */
struct MC_TASKS_DLLAPI MetaTaskLoader
{
  /*! \brief Register a new MetaTask loading function */
  using load_fun = std::function<MetaTaskPtr(mc_solver::QPSolver&, const mc_rtc::Configuration&)>;

  /*! \brief Register a new loading function
   *
   * \param type Type of MetaTask this function handles (should match the
   * "type" entry of the expected JSON objects)
   *
   * \param fn Function that will be added
   */
  static bool register_load_function(const std::string & type, load_fun fn);

  /*! \brief Load a MetaTask from a file
   *
   * \param solver Solver to create the task for
   *
   * \param file File to load the task from
   *
   * \throws If the file does not exist or the loaded JSON
   * object does not represent a known MetaTask
   */
  static MetaTaskPtr load(mc_solver::QPSolver & solver,
                          const std::string & file);

  /*! \brief Load a MetaTask from an mc_rtc::Configuration
   * entry
   *
   * \param solver Solver to create the task for
   *
   * \param config Configuration to load the task from
   *
   * \throws If the loaded JSON object does not represent
   * a known MetaTask
   */
  static MetaTaskPtr load(mc_solver::QPSolver & solver,
                          const mc_rtc::Configuration & config);

  /*! \brief Retrieve a more precise task type from a file
   *
   * \throws If the MetaTaskPtr load throws or if the
   * retrieve MetaTask is not of the requested type
   */
  template<typename T,
    typename std::enable_if<(!std::is_same<T, MetaTask>::value) && std::is_base_of<MetaTask, T>::value, int>::type = 0>
  static std::shared_ptr<T> load(mc_solver::QPSolver & solver,
                                 const std::string & file);

  /*! \brief Retrieve a more precise task type from an
   * mc_rtc::Configuration entry
   *
   * \throws If the MetaTaskPtr load throws or if the
   * retrieve MetaTask is not of the requested type
   */
  template<typename T,
    typename std::enable_if<(!std::is_same<T, MetaTask>::value) && std::is_base_of<MetaTask, T>::value, int>::type = 0>
  static std::shared_ptr<T> load(mc_solver::QPSolver & solver,
                                 const mc_rtc::Configuration & config);

private:
  template<typename T>
  static std::shared_ptr<T> cast(const MetaTaskPtr & mt);

  static std::map<std::string, load_fun> & get_fns();

  static std::unique_ptr<std::map<std::string, load_fun>> fns_ptr;
};

}

#include "MetaTaskLoader.hpp"
