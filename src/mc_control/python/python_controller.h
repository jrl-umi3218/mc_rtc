#include <mc_control/mc_controller.h>

namespace mc_rbdyn
{
typedef std::shared_ptr<RobotModule> RobotModulePtr;
}

extern "C"
{
  MC_CONTROL_DLLAPI void MC_RTC_CONTROLLER(std::vector<std::string> & names);
  MC_CONTROL_DLLAPI void destroy(mc_control::MCController * ptr);
  MC_CONTROL_DLLAPI mc_control::MCController * create(const std::string &,
                                                      const std::string & controller_name,
                                                      const std::shared_ptr<mc_rbdyn::RobotModule> & robot,
                                                      const double & dt,
                                                      const mc_control::Configuration &);
  MC_CONTROL_DLLAPI void LOAD_GLOBAL();
}
