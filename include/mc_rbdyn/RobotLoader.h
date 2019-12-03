/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/*! Interface used to load robots */

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/api.h>
#include <mc_rtc/config.h>
#include <mc_rtc/loader.h>

#include <memory>
#include <mutex>

namespace mc_rbdyn
{

namespace details
{

template<typename... Args>
struct are_strings : std::true_type {};

template<typename T>
struct are_strings<T> : std::is_same<std::string, T> {};

template<typename T, typename ... Args>
struct are_strings<T, Args...> : std::integral_constant<bool, are_strings<T>::value && are_strings<Args...>::value> {};

static_assert(are_strings<>::value, "OK");
static_assert(are_strings<std::string>::value, "OK");
static_assert(are_strings<std::string, std::string>::value, "OK");
static_assert(are_strings<std::string, std::string, std::string>::value, "OK");
static_assert(!are_strings<const char *>::value, "OK");
static_assert(!are_strings<std::string, std::string, bool>::value, "OK");

} // namespace

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
    static_assert(details::are_strings<Args...>::value, "get_robot_module expects all arguments to be std::string");
    std::lock_guard<std::mutex> guard{mtx};
    init();
    auto rm = robot_loader->create_object(name, args...);
    rm->_parameters = {name};
    fill_rm_parameters(rm, args...);
    return rm;
  }

  /** Add additional directories to the robot module path
   * \param paths Directories to be added to the module path
   */
  static inline void update_robot_module_path(const std::vector<std::string> & paths)
  {
    std::lock_guard<std::mutex> guard{mtx};
    init();
    robot_loader->load_libraries(paths);
  }

  /** Remove all loaded libraries */
  static inline void clear()
  {
    std::lock_guard<std::mutex> guard{mtx};
    init(true);
    robot_loader->clear();
  }

  /** Check if a robot is available
   * \param name Robot name
   */
  static bool has_robot(const std::string & name)
  {
    std::lock_guard<std::mutex> guard{mtx};
    init();
    return robot_loader->has_object(name);
  }

  /** Enable robot's creation sandboxing
   * \param enable_sandbox If true, robot's create call are sandboxed
   */
  static void enable_sandboxing(bool enable_sandbox)
  {
    std::lock_guard<std::mutex> guard{mtx};
    enable_sandbox_ = enable_sandbox;
    if(robot_loader)
    {
      robot_loader->enable_sandboxing(enable_sandbox_);
    }
  }

  static void set_verbosity(bool verbose)
  {
    std::lock_guard<std::mutex> guard{mtx};
    verbose_ = verbose;
    if(robot_loader)
    {
      robot_loader->set_verbosity(verbose);
    }
  }

  /** Returns a list of available robots */
  static std::vector<std::string> available_robots()
  {
    std::lock_guard<std::mutex> guard{mtx};
    init();
    return robot_loader->objects();
  }

private:
  static inline void init(bool skip_default_path = false)
  {
    if(!robot_loader)
    {
      try
      {
        std::vector<std::string> default_path = {};
        if(!skip_default_path)
        {
          default_path.push_back(mc_rtc::MC_ROBOTS_INSTALL_PREFIX);
        }
        robot_loader.reset(new mc_rtc::ObjectLoader<mc_rbdyn::RobotModule>("MC_RTC_ROBOT_MODULE", default_path,
                                                                           enable_sandbox_, verbose_));
      }
      catch(const mc_rtc::LoaderException & exc)
      {
        LOG_ERROR("Failed to initialize RobotLoader: " << exc.what())
        throw(exc);
      }
    }
  }

  template<typename... Args>
  static void fill_rm_parameters(mc_rbdyn::RobotModulePtr & rm, const std::string & arg0, const Args &... args)
  {
    rm->_parameters.push_back(arg0);
    fill_rm_parameters(rm, args...);
  }

  static void fill_rm_parameters(mc_rbdyn::RobotModulePtr &) {}

  static std::unique_ptr<mc_rtc::ObjectLoader<mc_rbdyn::RobotModule>> robot_loader;
  static bool enable_sandbox_;
  static bool verbose_;
  static std::mutex mtx;
};

} // namespace mc_rbdyn
