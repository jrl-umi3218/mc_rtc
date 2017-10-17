#include <boost/test/unit_test.hpp>

#include <mc_control/mc_fsm_state_factory.h>

const std::string FSM_STATES_DIR = "@CMAKE_CURRENT_BINARY_DIR@/fsm_states/";
const std::string FSM_STATES_JSON_DIR = "@CMAKE_CURRENT_SOURCE_DIR@/fsm_states/data/";

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
