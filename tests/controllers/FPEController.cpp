/*! This library raises an FPE */
#include <mc_control/mc_controller.h>

#include <random>

struct FPEController : public mc_control::MCController
{
  FPEController(const std::shared_ptr<mc_rbdyn::RobotModule> & rm, const double & dt, const mc_control::Configuration &)
   : mc_control::MCController(rm, dt)
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    // Make sure we get a = 0 but should also make sure the compiler does not find out
    std::uniform_int_distribution<int> d(0, 0);
    int a = d(gen);
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
  CONTROLLER_MODULE_API mc_control::MCController * create(const std::string&, const std::shared_ptr<mc_rbdyn::RobotModule> & rm, const double & dt, const mc_control::Configuration & conf)
  {
    return new FPEController(rm, dt, conf);
  }
}
