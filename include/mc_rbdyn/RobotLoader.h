/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/*! Interface used to load robots */

#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotConverter.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/io_utils.h>

#include <mc_rtc/config.h>
#include <mc_rtc/loader.h>

#include <map>
#include <memory>
#include <mutex>

namespace mc_rbdyn
{

namespace details
{

template<typename... Args>
struct are_strings : std::true_type
{
};

template<typename T>
struct are_strings<T> : std::is_same<std::string, T>
{
};

template<typename T, typename... Args>
struct are_strings<T, Args...> : std::integral_constant<bool, are_strings<T>::value && are_strings<Args...>::value>
{
};

static_assert(are_strings<>::value, "OK");
static_assert(are_strings<std::string>::value, "OK");
static_assert(are_strings<std::string, std::string>::value, "OK");
static_assert(are_strings<std::string, std::string, std::string>::value, "OK");
static_assert(!are_strings<const char *>::value, "OK");
static_assert(!are_strings<std::string, std::string, bool>::value, "OK");

// Construct a string from a provided argument if it's not one, otherwise just return the string
template<typename T>
typename std::conditional<std::is_same<std::string, T>::value, const std::string &, std::string>::type to_string(
    const T & value)
{
  static_assert(!std::is_integral<T>::value,
                "Passing integral value here would create empty strings of the provided size");
  return value;
}

} // namespace details

/*! \class RobotLoader
 * \brief Load RobotModule instances from shared libraries
 */
struct MC_RBDYN_DLLAPI RobotLoader
{
public:
  /** Return a RobotModule constructed with the provided Args
   * \param name The module name
   * \param args The arguments provided to the module creation function
   * \note
   * It is the responsability of the caller to make sure that the signature of the module creation fits that declared by
   * the module
   */
  template<typename... Args>
  static mc_rbdyn::RobotModulePtr get_robot_module(const std::string & name, const Args &... args)
  {
    if(!details::are_strings<Args...>::value) { return get_robot_module(name, details::to_string(args)...); }
    std::unique_lock<std::recursive_mutex> guard{mtx};
    init();
    mc_rbdyn::RobotModulePtr rm = nullptr;
    auto setup_canonical = [](mc_rbdyn::RobotModulePtr rm)
    {
      assert(rm);
      if(rm->_canonicalParameters.empty()) { rm->_canonicalParameters = rm->_parameters; }
      if(!rm->controlToCanonicalPostProcess)
      {
        rm->controlToCanonicalPostProcess = [](const mc_rbdyn::Robot &, mc_rbdyn::Robot &) {};
      }
    };
    if(aliases.count(name))
    {
      const auto & params = aliases[name];
      if(params.size() == 1) { rm = get_robot_module_from_lib(params[0]); }
      else if(params.size() == 2) { rm = get_robot_module_from_lib(params[0], params[1]); }
      else if(params.size() == 3) { rm = get_robot_module_from_lib(params[0], params[1], params[2]); }
      else
      {
        mc_rtc::log::error_and_throw<mc_rtc::LoaderException>(
            "Aliases can only handle 1 to 3 parameters, {} provided ({})", params.size(),
            mc_rtc::io::to_string(params));
      }
      rm->_parameters.resize(1);
      rm->_parameters[0] = name;
    }
    else { rm = get_robot_module_from_lib(name, args...); }
    setup_canonical(rm);
    return rm;
  }

  /** Returns the RobotModule that would be loaded from get_robot_module(args[0], ..., args[args.size() - 1])
   *
   * This is arbitrarly limited to 3 arguments
   */
  static RobotModulePtr get_robot_module(const std::vector<std::string> & args);

  template<typename RetT, typename... Args>
  static void register_object(const std::string & name, std::function<RetT *(const Args &...)> callback)
  {
    std::unique_lock<std::recursive_mutex> guard{mtx};
    init();
    robot_loader->register_object(name, callback);
  }

  /** Add additional directories to the robot module path
   * \param paths Directories to be added to the module path
   */
  static void update_robot_module_path(const std::vector<std::string> & paths);

  /** Remove all loaded libraries */
  static inline void clear()
  {
    std::lock_guard<std::recursive_mutex> guard{mtx};
    init(true);
    robot_loader->clear();
    aliases.clear();
  }

  /** Check if a robot is available
   * \param name Robot name
   */
  static inline bool has_robot(const std::string & name)
  {
    std::lock_guard<std::recursive_mutex> guard{mtx};
    init();
    return robot_loader->has_object(name) || aliases.count(name) != 0;
  }

  static inline void set_verbosity(bool verbose)
  {
    std::lock_guard<std::recursive_mutex> guard{mtx};
    verbose_ = verbose;
    if(robot_loader) { robot_loader->set_verbosity(verbose); }
  }

  /** Returns a list of available robots */
  static std::vector<std::string> available_robots();

  /** Load aliases
   *
   * An aliases file should be a map of alias to param vector
   *
   * \param fname A JSON or YAML containing aliases
   *
   */
  static void load_aliases(const std::string & fname);

private:
  static void init(bool skip_default_path = false);

  template<typename... Args>
  static void fill_rm_parameters(mc_rbdyn::RobotModulePtr & rm, const std::string & arg0, const Args &... args)
  {
    rm->_parameters.push_back(arg0);
    fill_rm_parameters(rm, args...);
  }

  static inline void fill_rm_parameters(mc_rbdyn::RobotModulePtr &) {}

  template<typename... Args>
  static mc_rbdyn::RobotModulePtr get_robot_module_from_lib(const std::string & name, const Args &... args)
  {
    if(!robot_loader->has_object(name))
    {
      mc_rtc::log::error("Cannot load the requested robot: {}\nIt is neither a valid alias nor a known exported robot",
                         name);
      mc_rtc::log::info("Available robots:");
      mtx.unlock();
      for(const auto & r : available_robots()) { mc_rtc::log::info("- {}", r); }
      mc_rtc::log::error_and_throw<mc_rtc::LoaderException>("Cannot load the requested robot: {}", name);
    }
    mc_rbdyn::RobotModulePtr rm = robot_loader->create_object(name, args...);
    if(!rm) { mc_rtc::log::error_and_throw("Failed to load {}", name); }
    rm->_parameters = {name};
    fill_rm_parameters(rm, args...);
    return rm;
  }

  static std::unique_ptr<mc_rtc::ObjectLoader<mc_rbdyn::RobotModule>> robot_loader;
  static bool verbose_;
  /**
   * We use a recursive mutex here to allow nested calls to get_robot_module()
   * In particular, this allows calling get_robot_module from within the create() function of a robot module library.
   * This is particularely useful to take advantage of RobotModule::connect to expose new first-class named connected
   *modules directly from the robot module library.
   **/
  static std::recursive_mutex mtx;
  static std::map<std::string, std::vector<std::string>> aliases;
}; // namespace mc_rbdyn

} // namespace mc_rbdyn
