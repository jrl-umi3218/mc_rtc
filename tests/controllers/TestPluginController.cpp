/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif
#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestPluginController : public MCController
{
public:
  TestPluginController(const std::string & name, mc_rbdyn::RobotModulePtr rm, double dt)
  : MCController(rm, dt), name_(name)
  {
    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(dynamicsConstraint);
    solver().addTask(postureTask.get());
    solver().setContacts(
        {mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"), mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")});
    mc_rtc::log::success("Created {}", name_);
  }

  bool run() override
  {
    // The static map here is used to pass data across controller boundaries
    static std::map<std::string, void *> plugin_addresses;
    if(nrIter == 0)
    {
      for(const auto & plugin : common_plugins_)
      {
        BOOST_REQUIRE(datastore().has(plugin + "::address"));
        BOOST_REQUIRE(plugin_addresses.count(plugin));
        void * p_addr = datastore().get<void *>(plugin + "::address");
        BOOST_REQUIRE(plugin_addresses[plugin] == p_addr);
      }
      for(const auto & plugin : expected_plugins_)
      {
        BOOST_REQUIRE(datastore().has(plugin + "::address"));
        void * p_addr = datastore().get<void *>(plugin + "::address");
        plugin_addresses[plugin] = p_addr;
        BOOST_REQUIRE(datastore().has(plugin + "::config"));
        auto plugin_c = datastore().get<mc_rtc::Configuration>(plugin + "::config");
        BOOST_REQUIRE(plugin_c.has("common"));
        std::string plugin_c_common = plugin_c("common");
        BOOST_REQUIRE_EQUAL(plugin_c_common, plugin);
        BOOST_REQUIRE(plugin_c.has(name_));
        BOOST_REQUIRE(plugin_c(name_));
        BOOST_REQUIRE(plugin_c.has("exclusive_to_TestPluginController_1_2"));
        bool exclusive = plugin_c("exclusive_to_TestPluginController_1_2");
        if(name_ == "TestPluginController_1_2")
        {
          BOOST_REQUIRE_EQUAL(exclusive, plugin == "Plugin1");
        }
        else
        {
          BOOST_REQUIRE_EQUAL(exclusive, false);
        }
      }
    }
    bool ret = MCController::run();
    nrIter++;
    for(const auto & plugin : expected_plugins_)
    {
      BOOST_REQUIRE(datastore().has(plugin + "::iter_before"));
      BOOST_REQUIRE(datastore().get<size_t>(plugin + "::iter_before") == nrIter);
      BOOST_REQUIRE(datastore().has(plugin + "::iter_after"));
      BOOST_REQUIRE(datastore().get<size_t>(plugin + "::iter_after") == nrIter - 1);
    }
    return ret;
  }

  // The plugin that we expect to be running in this controller
  std::vector<std::string> expected_plugins_ = {};
  // The plugin we expect to have kept from the previous controller
  std::vector<std::string> common_plugins_ = {};

private:
  std::string name_;
  size_t nrIter = 0;
};

} // namespace mc_control

extern "C"
{

  CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> & names)
  {
    CONTROLLER_CHECK_VERSION("TestPluginController")
    names = {"TestPluginController_1_2", "TestPluginController_2_3"};
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
                                                          const mc_rbdyn::RobotModulePtr & rm,
                                                          const double & dt,
                                                          const mc_rtc::Configuration &)
  {
    auto out = new mc_control::TestPluginController(name, rm, dt);
    if(name == "TestPluginController_1_2")
    {
      out->expected_plugins_ = {"Plugin0", "Plugin1", "Plugin2"};
    }
    else if(name == "TestPluginController_2_3")
    {
      out->expected_plugins_ = {"Plugin0", "Plugin2", "Plugin3"};
      out->common_plugins_ = {"Plugin0", "Plugin2"};
    }
    else
    {
      mc_rtc::log::critical("UNREACHABLE");
      return nullptr;
    }
    return out;
  }
}
