/*! This library voluntarily contains unresolved symbols */
#include <mc_control/mc_controller.h>

struct NotImplemented : public mc_control::MCController
{
  NotImplemented();
};

extern "C"
{
  CONTROLLER_MODULE_API const char * CLASS_NAME() { return "UnresolvedSymbolController"; }
  CONTROLLER_MODULE_API void destroy(mc_control::MCController * ptr)
  {
    delete ptr;
  }
  CONTROLLER_MODULE_API mc_control::MCController * create(const std::shared_ptr<mc_rbdyn::RobotModule>&, const double&, const mc_control::Configuration&)
  {
    return new NotImplemented();
  }
}
