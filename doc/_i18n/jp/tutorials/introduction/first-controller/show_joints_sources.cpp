#include <mc_rbdyn/RobotLoader.h>
int main()
{
  // 以下のロボットモジュールの名前を自分のロボットのものに変える
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
