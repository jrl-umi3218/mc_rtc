#include <boost/test/unit_test.hpp>

#include <mc_rbdyn/RobotLoader.h>

#include <mc_control/mc_fsm_state_factory.h>
#include <mc_control/mc_fsm_controller.h>

#include "@CMAKE_CURRENT_SOURCE_DIR@/fsm_states/ConfigureState.h"

const std::string FSM_STATES_DIR = "@CMAKE_CURRENT_BINARY_DIR@/fsm_states/";
const std::string FSM_STATES_JSON_DIR = "@CMAKE_CURRENT_SOURCE_DIR@/fsm_states/data/";

mc_control::FSMController & get_default_controller()
{
  static std::shared_ptr<mc_control::FSMController> ctl_ptr = nullptr;
  if(ctl_ptr)
  {
    return *ctl_ptr;
  }
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({"@CMAKE_CURRENT_BINARY_DIR@/../src/mc_robots"});
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("HRP2DRC");
  mc_rtc::Configuration config;
  config.add("Managed", true);
  ctl_ptr = std::make_shared<mc_control::FSMController>(rm, 0.005, config);
  return *ctl_ptr;
}

void check_state(mc_control::FSMStateFactory & factory,
                 const std::string & state,
                 const std::vector<std::string> & outputs)
{
  BOOST_REQUIRE(factory.hasState(state));
  BOOST_REQUIRE(factory.stateOutputs(state).size() == outputs.size());
  for(const auto & o : outputs)
  {
    BOOST_REQUIRE(factory.isValidOutput(state, o));
  }
}

void check_states(mc_control::FSMStateFactory & factory,
                  const std::map<std::string, std::vector<std::string>> & states)
{
  BOOST_REQUIRE(factory.states().size() == states.size());
  for(const auto & s : states)
  {
    check_state(factory, s.first, s.second);
  }
}

BOOST_AUTO_TEST_CASE(TestSingleStateLoading)
{
  mc_control::FSMStateFactory factory {
    {FSM_STATES_DIR + "SingleState"},
    {},
    false
  };
  check_states(factory,
               {
                {"SingleState", {"OK"}}
               });
}

BOOST_AUTO_TEST_CASE(TestJsonInheritanceSingle)
{
  mc_control::FSMStateFactory factory {
    {FSM_STATES_DIR + "SingleState"},
    {FSM_STATES_JSON_DIR + "SingleState.json"},
    false
  };
  check_states(factory,
               {
                {"SingleState", {"OK"}},
                {"SingleStateBis", {"OK"}}
               });
}

BOOST_AUTO_TEST_CASE(TestJsonInheritanceMultiple)
{
  mc_control::FSMStateFactory factory {
    {FSM_STATES_DIR + "SingleState"},
    {FSM_STATES_JSON_DIR + "SingleStateMulti.json"},
    false
  };
  check_states(factory,
               {
                {"SingleState", {"OK"}},
                {"SingleState1", {"OK"}},
                {"SingleState2", {"OK"}},
                {"SingleState3", {"OK"}},
                {"SingleState4", {"OK"}}
               });
}

BOOST_AUTO_TEST_CASE(TestJsonInheritanceSplit)
{
  mc_control::FSMStateFactory factory {
    {FSM_STATES_DIR + "SingleState"},
    {
      FSM_STATES_JSON_DIR + "SingleState1.json",
      FSM_STATES_JSON_DIR + "SingleState2.json",
      FSM_STATES_JSON_DIR + "SingleState3.json",
      FSM_STATES_JSON_DIR + "SingleState4.json"
    },
    false
  };
  check_states(factory,
               {
                {"SingleState", {"OK"}},
                {"SingleState1", {"OK"}},
                {"SingleState2", {"OK"}},
                {"SingleState3", {"OK"}},
                {"SingleState4", {"OK"}}
               });
}

BOOST_AUTO_TEST_CASE(TestMultipleStatesLoading)
{
  mc_control::FSMStateFactory factory {
    {FSM_STATES_DIR + "MultipleStates"},
    {},
    false
  };
  check_states(factory,
               {
                {"State1", {"OK"}},
                {"State2", {"OK", "NOK", "FAILURE"}}
               });
}

BOOST_AUTO_TEST_CASE(TestConfigureState)
{
  mc_control::FSMStateFactory factory {
    {FSM_STATES_DIR + "ConfigureState"},
    {FSM_STATES_JSON_DIR + "ConfigureState.json"},
    false
  };
  check_states(factory,
               {
                {"ConfigureState", {"OK"}},
                {"ConfigureState2", {"OK"}},
                {"ConfigureState4", {"OK"}},
                {"ConfigureState8", {"OK"}}
               });
  auto & ctl = get_default_controller();
  auto test_state = [&factory, &ctl](const std::string & name, unsigned int value, const mc_rtc::Configuration & config)
  {
    auto state_base = factory.create(name, ctl, config);
    auto state = std::dynamic_pointer_cast<ConfigureState>(state_base);
    BOOST_REQUIRE(state != nullptr);
    BOOST_REQUIRE(state->value() == value);
  };
  test_state("ConfigureState", 0, {});
  test_state("ConfigureState2", 2, {});
  test_state("ConfigureState4", 4, {});
  test_state("ConfigureState8", 8, {});
  mc_rtc::Configuration config;
  config.add("value", 42);
  test_state("ConfigureState8", 42, config);
}
