#include <Eigen/Core>

#include <tcp_control/hrp2_data.h>
#include <tcp_control/hrp4_data.h>

#include "MCControlTCP.hxx"

int main(int argc, char **argv)
{
  std::string robot = "hrp4";

  if (robot == "hrp2" || robot == "hrp2_10" || robot == "hrp2_10_flex")
  {
    MCControlTCP<HRP2OpenHRPSensors, HRP2OpenHRPControl> nodeWrapper;
    nodeWrapper.initialize(argc, argv);
    nodeWrapper.start("HRP2JRL");
  }
  else if (robot == "hrp4" || robot == "hrp4_fixed_hands")
  {
    MCControlTCP<HRP4OpenHRPSensors, HRP4OpenHRPControl> nodeWrapper;
    nodeWrapper.initialize(argc, argv);
    nodeWrapper.start("HRP4LIRMM");
  }
  else
  {
    return -1;
  }

  return 0;
}
