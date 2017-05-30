/*! This library raises an FPE */
#include <mc_control/mc_controller.h>

struct FPEController : public mc_control::MCController
{
  FPEController(const std::shared_ptr<mc_rbdyn::RobotModule> & rm, const double & dt, const mc_control::Configuration &)
   : mc_control::MCController(rm, dt)
  {
    int a = 0;
    b = 42/a;
  }
  int b;
};

extern "C"
{
  CONTROLLER_MODULE_API std::vector<std::string> MC_RTC_CONTROLLER() { return {"FPEController"}; }
  CONTROLLER_MODULE_API void destroy(mc_control::MCController * ptr)
  {
    delete ptr;
  }
  CONTROLLER_MODULE_API mc_control::MCController * create(const std::shared_ptr<mc_rbdyn::RobotModule> & rm, const double & dt, const mc_control::Configuration & conf)
  {
    return new FPEController(rm, dt, conf);
  }
}
