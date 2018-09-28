#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>
#include <mc_solver/QPSolver.h>

namespace mc_solver
{

/** Retrieve an object using a JSON configuration object and a solver
 * instance
 *
 * Other objects can register new types that can be loaded through such an
 * interface. If you are writing a new mc_tasks::MetaTask or
 * mc_solver::ConstraintSet you will probably register through
 * mc_tasks::MetaTaskLoader and mc_solver::ConstraintSetLoader
 * respectively.
 *
 * The CRTP is used to provide an ODR-resilient storage here.
 */
template<typename Derived, typename T>
struct GenericLoader
{
  /** shared_ptr to T */
  using T_ptr = std::shared_ptr<T>;

  /** A function that is able to load a T object through a solver instance
   * and JSON configuration */
  using load_fun = std::function<T_ptr(mc_solver::QPSolver &, const mc_rtc::Configuration &)>;

  /** Storage type, actual storage location is returned by Derived::storage() */
  using storage_t = std::map<std::string, load_fun>;

  /** Register a new loading function
   *
   * \param type Type of the object this function handles. It should match
   * the "type" entry of the expected JSON objects
   *
   * \param fn Function that will be registered
   */
  static bool register_load_function(const std::string & type, load_fun fn);

  /** Load an object from a file
   *
   * \param solver Solver to create the object for
   *
   * \param file File to load the object from
   *
   * \throws If the file does not exist or the loaded JSON object does not
   * represent a known object
   */
  static T_ptr load(mc_solver::QPSolver & solver, const std::string & file);

  /** Load an object from an mc_rtc::Configuration object
   *
   * \param solver Solver to create the object for
   *
   * \param config Configuration to load the object from
   *
   * \throws If the loaded JSON object does not represent a known object
   */
  static T_ptr load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config);

  /** Retrieve a more precise object's type from a file */
  template<typename U,
           typename std::enable_if<(!std::is_same<U, T>::value) && std::is_base_of<T, U>::value, int>::type = 0>
  static std::shared_ptr<U> load(mc_solver::QPSolver & solver, const std::string & file);

  /** Retrieve a more precise object's type from a Configuration entry */
  template<typename U,
           typename std::enable_if<(!std::is_same<U, T>::value) && std::is_base_of<T, U>::value, int>::type = 0>
  static std::shared_ptr<U> load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config);

private:
  template<typename U>
  static std::shared_ptr<U> cast(const T_ptr & p);

  static storage_t & get_fns();
};

} // namespace mc_solver

#include "GenericLoader.hpp"
