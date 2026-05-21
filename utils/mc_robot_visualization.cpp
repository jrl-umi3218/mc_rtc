#include <mc_control/mc_global_controller.h>
#include "RobotVisualizer.h"

int main(int argc, char * argv[])
{
  // Update runtime paths (robot modules, etc)
  // This is required in nix to ensure that plugins installed in different store location are available
  // e.g a robot module is installed in its own store prefix, and mc_rtc is made aware of it through RobotModulePaths
  auto gconfig = mc_control::MCGlobalController::GlobalConfiguration{""};

  RobotVisualizer visualizer(params_from_main(argc, argv), false, false);
  visualizer.run();
  return 0;
}
