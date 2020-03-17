/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/StateFactory.h>
#include <mc_rbdyn/RobotLoader.h>

#include <boost/test/unit_test.hpp>

#include "testFSMStateFactoryConfig.h"

mc_control::fsm::Controller & get_default_controller()
{
  static std::shared_ptr<mc_control::fsm::Controller> ctl_ptr = nullptr;
  if(ctl_ptr)
  {
    return *ctl_ptr;
  }
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  mc_rtc::Configuration config;
  config.add("Managed", true);
  ctl_ptr = std::make_shared<mc_control::fsm::Controller>(rm, 0.005, config);
  return *ctl_ptr;
}

void check_state(mc_control::fsm::StateFactory & factory, const std::string & state)
{
  BOOST_REQUIRE(factory.hasState(state));
}

void check_states(mc_control::fsm::StateFactory & factory, const std::vector<std::string> & states)
{
  BOOST_REQUIRE(factory.states().size() == states.size());
  for(const auto & s : states)
  {
    check_state(factory, s);
  }
}

BOOST_AUTO_TEST_CASE(TestSingleStateLoading)
{
  mc_control::fsm::StateFactory factory{{SingleState_DIR}, {}, false};
  check_states(factory, {"SingleState"});
}

BOOST_AUTO_TEST_CASE(TestJsonInheritanceSingle)
{
  mc_control::fsm::StateFactory factory{{SingleState_DIR}, {FSM_STATES_JSON_DIR + "SingleState.json"}, false};
  check_states(factory, {"SingleState", "SingleStateBis"});
}

BOOST_AUTO_TEST_CASE(TestJsonInheritanceMultiple)
{
  mc_control::fsm::StateFactory factory{{SingleState_DIR}, {FSM_STATES_JSON_DIR + "SingleStateMulti.json"}, false};
  check_states(factory, {
                            "SingleState",
                            "SingleState1",
                            "SingleState2",
                            "SingleState3",
                            "SingleState4",
                        });
}

BOOST_AUTO_TEST_CASE(TestJsonInheritanceSplit)
{
  mc_control::fsm::StateFactory factory{
      {SingleState_DIR},
      {FSM_STATES_JSON_DIR + "SingleState1.json", FSM_STATES_JSON_DIR + "SingleState2.json",
       FSM_STATES_JSON_DIR + "SingleState3.json", FSM_STATES_JSON_DIR + "SingleState4.json"},
      false};
  check_states(factory, {
                            "SingleState",
                            "SingleState1",
                            "SingleState2",
                            "SingleState3",
                            "SingleState4",
                        });
}

BOOST_AUTO_TEST_CASE(TestMultipleStatesLoading)
{
  mc_control::fsm::StateFactory factory{{MultipleStates_DIR}, {}, false};
  check_states(factory, {"State1", "State2"});
}

BOOST_AUTO_TEST_CASE(TestConfigureState)
{
  mc_control::fsm::StateFactory factory{{ConfigureState_DIR}, {FSM_STATES_JSON_DIR + "ConfigureState.json"}, false};
  check_states(factory, {
                            "ConfigureState",
                            "ConfigureState2",
                            "ConfigureState4",
                            "ConfigureState8",
                        });
  auto & ctl = get_default_controller();
  auto test_state = [&factory, &ctl](const std::string & name, unsigned int value,
                                     const mc_rtc::Configuration & config) {
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

BOOST_AUTO_TEST_CASE(TestTransitionMap)
{
  mc_control::fsm::StateFactory factory{{MultipleStates_DIR}, {}, false};
  check_states(factory, {"State1", "State2"});
  { // Test single state with multiple outputs to the same state
    // This should only add the transition once to the transitions() list
    mc_rtc::Configuration tConfig;
    tConfig.add("transitions", std::vector<std::vector<std::string>>{{"State1", "Output1", "State2"},
                                                                     {"State1", "Output2", "State2"},
                                                                     {"State2", "Output1", "State1"},
                                                                     {"State2", "Output2", "State1"}});
    mc_control::fsm::TransitionMap transitions;
    transitions.init(factory, tConfig);
    BOOST_REQUIRE(transitions.transitions("State1") == std::unordered_set<std::string>{"State2"});
    BOOST_REQUIRE(transitions.transitions("State2") == std::unordered_set<std::string>{"State1"});
  }
  { // Test wrongly defined transition with the same output going to multiple states
    // This should display a warning message, and use the latest defined
    // transition
    mc_rtc::Configuration tConfig;
    tConfig.add("transitions",
                std::vector<std::vector<std::string>>{{"State1", "Output", "State1"}, {"State1", "Output", "State2"}});
    mc_control::fsm::TransitionMap transitions;
    transitions.init(factory, tConfig);
    BOOST_REQUIRE(transitions.transitions("State1") == std::unordered_set<std::string>{"State2"});
  }
}
