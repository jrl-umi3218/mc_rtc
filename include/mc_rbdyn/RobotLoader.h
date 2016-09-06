#pragma once

/*! Interface used to load robots */

#include <memory>

#include <mc_rbdyn/api.h>
#include <mc_rbdyn/RobotModule.h>

#include <mc_rtc/config.h>
#include <mc_rtc/loader.h>

namespace mc_rbdyn
{

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
   * It is the responsability of the caller to make sure that the signature of the module creation fits that declared by the module
  */
  template<typename ... Args>
  static std::shared_ptr<mc_rbdyn::RobotModule> get_robot_module(const std::string & name, const Args & ... args)
  {
    init();
    return robot_loader->create_object(name, args...);
  }

  /** Add additional directories to the robot module path
   * \param paths Directories to be added to the module path
  */
  static inline void update_robot_module_path(const std::vector<std::string> & paths)
  {
    init();
    robot_loader->load_libraries(paths);
  }

  /** Remove all loaded libraries */
  static inline void clear()
  {
    init();
    robot_loader->clear();
  }

  /** Check if a robot is available
   * \param name Robot name
   */
  static bool has_robot(const std::string & name)
  {
    init();
    return robot_loader->has_object(name);
  }

  /** Enable robot's creation sandboxing
   * \param enable_sandbox If true, robot's create call are sandboxed
   */
  static void enable_sandboxing(bool enable_sandbox)
  {
    init();
    robot_loader->enable_sandboxing(enable_sandbox);
  }
private:
  static inline void init()
  {
    if(!robot_loader)
    {
      try
      {
        robot_loader.reset(new mc_rtc::ObjectLoader<mc_rbdyn::RobotModule>({mc_rtc::MC_ROBOTS_INSTALL_PREFIX}, true));
      }
      catch(const mc_rtc::LoaderException & exc)
      {
        LOG_ERROR("Failed to initialize RobotLoader")
        throw(exc);
      }
    }
  }
  static std::unique_ptr<mc_rtc::ObjectLoader<mc_rbdyn::RobotModule>> robot_loader;
};

} // namespace mc_rbdyn
