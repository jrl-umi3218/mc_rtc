#include <mc_rbdyn/RobotLoader.h>
int main()
{
  // Here you can change the robot module name with your own robot
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  for(const auto & j : rm->mb.joints())
  {
    if(j.dof() == 1 && !j.isMimic())
    {
      std::cout << "- " << j.name() << "\n";
    }
  }
  return 0;
}
