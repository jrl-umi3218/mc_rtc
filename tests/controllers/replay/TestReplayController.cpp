/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif
#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

#include <utils.h>

namespace mc_control
{

static std::string REPLAY_PATH = "";

static auto calib = []()
{
  mc_rbdyn::detail::ForceSensorCalibData calib;
  calib.mass = 42.42;
  calib.worldForce = random_fv();
  calib.X_f_ds = random_pt();
  calib.X_p_vb = random_pt();
  calib.offset = random_fv();
  return calib;
}();

template<bool Play>
struct MC_CONTROL_DLLAPI TestReplayController : public MCController
{
public:
  TestReplayController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & cfg)
  : MCController(rm, dt, cfg)
  {
    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(kinematicsConstraint);
    solver().addConstraintSet(selfCollisionConstraint);
    solver().addConstraintSet(compoundJointConstraint);
    postureTask->stiffness(200.0);
    qpsolver->setContacts({});
    if constexpr(!Play)
    {
      const_cast<mc_rbdyn::ForceSensor &>(robot().forceSensor("LeftFootForceSensor")).loadCalibrator(calib);
    }
  }

  bool run() override
  {
    if constexpr(Play)
    {
      BOOST_REQUIRE(robot().forceSensor("LeftFootForceSensor").calib() == calib);
      BOOST_REQUIRE(datastore().has("Replay::Log"));
      BOOST_REQUIRE(!datastore().has("NOT_IN_DATASTORE"));
    }
    else { BOOST_REQUIRE(!datastore().has("Replay::Log")); }
    bool ret = MCController::run();
    iters_ += 1;
    if constexpr(Play)
    {
      BOOST_REQUIRE(datastore().has("iter"));
      uint64_t log_iter = datastore().template get<uint64_t>("iter");
      if(log_iter != iters_) { mc_rtc::log::critical("{} != {}", log_iter, iters_); }
      BOOST_REQUIRE(log_iter == iters_);

      BOOST_REQUIRE(datastore().has("RENAMED_T"));
      double renamed_t = datastore().template get<double>("RENAMED_T");
      double expected_t = static_cast<double>(log_iter - 1) * timeStep;
      bool test_t = std::fabs(expected_t - renamed_t) < 1e-6;
      if(!test_t) { mc_rtc::log::critical("{} != {}", expected_t, renamed_t); }
      BOOST_REQUIRE(test_t);
    }
    return ret;
  }

  void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    if constexpr(Play)
    {
      BOOST_REQUIRE(!REPLAY_PATH.empty());
      auto log = std::make_shared<mc_rtc::log::FlatLog>(REPLAY_PATH);
      datastore().template make<decltype(log)>("Replay::Log", log);
    }
    else
    {
      REPLAY_PATH = logger().path();
      logger().addLogEntry("iter", [this]() { return iters_; });
    }
  }

  void stop() override
  {
    if constexpr(!Play) { logger().flush(); }
  }

private:
  uint64_t iters_ = 0;
};

} // namespace mc_control

extern "C"
{

  CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> & names)
  {
    names = {"TestReplayController_Record", "TestReplayController_Play"};
  }

  CONTROLLER_MODULE_API void destroy(mc_control::MCController * ptr)
  {
    delete ptr;
  }

  CONTROLLER_MODULE_API unsigned int create_args_required()
  {
    return 4;
  }

  CONTROLLER_MODULE_API mc_control::MCController * create(const std::string & name,
                                                          const mc_rbdyn::RobotModulePtr & robot,
                                                          const double & dt,
                                                          const mc_control::Configuration & cfg)
  {
    if(name == "TestReplayController_Record") { return new mc_control::TestReplayController<false>(robot, dt, cfg); }
    else
    {
      assert(name == "TestReplayController_Play");
      return new mc_control::TestReplayController<true>(robot, dt, cfg);
    }
  }
}
