/*! This library segfaults */
#include <mc_control/mc_controller.h>

struct SegfaultController : public mc_control::MCController
{
  SegfaultController(const std::shared_ptr<mc_rbdyn::RobotModule> & rm, const double & dt, const mc_control::Configuration &)
   : mc_control::MCController(rm, dt)
  {
    *(int*)0 = 0;
  }
};

extern "C"
{
  CONTROLLER_MODULE_API std::vector<std::string> MC_RTC_CONTROLLER() { return {"SegfaultController"}; }
  CONTROLLER_MODULE_API void destroy(mc_control::MCController * ptr)
  {
    delete ptr;
  }
  CONTROLLER_MODULE_API mc_control::MCController * create(const std::string&, const std::shared_ptr<mc_rbdyn::RobotModule> & rm, const double & dt, const mc_control::Configuration & conf)
  {
    return new SegfaultController(rm, dt, conf);
  }
}
