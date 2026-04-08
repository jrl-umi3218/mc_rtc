#include "TestMetaContinuity.h"
#include <mc_control/fsm/Controller.h>

#include "./TestMetaContinuity.h"

void TestMetaContinuity::start(mc_control::fsm::Controller & ctl)
{
  run(ctl);
}

bool TestMetaContinuity::run(mc_control::fsm::Controller & ctl)
{
  if(stateIter_++ >= 5)
  {
    output("OK");
    return true;
  }
  ++ctl.datastore().get<unsigned>("StateIter");
  return false;
}

void TestMetaContinuity::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("TestMetaContinuity", TestMetaContinuity)
