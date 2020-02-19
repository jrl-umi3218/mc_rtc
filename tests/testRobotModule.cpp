#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <Eigen/StdVector>

int main()
{

  auto env = mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                     std::string("ground"));
  auto hrp2 = mc_rbdyn::RobotLoader::get_robot_module("HRP2DRC");
  auto jvrc = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  {
    std::cout << "Loading env" << std::endl;
    auto robots = mc_rbdyn::loadRobots({env});
    std::cout << robots->robot().name() << std::endl;
    std::cout << "env OK\n" << std::endl;
  }
  {
    std::cout << "Loading jvrc" << std::endl;
    auto robots = mc_rbdyn::loadRobots({jvrc});
    std::cout << robots->robot().name() << std::endl;
    std::cout << "jvrc OK\n" << std::endl;
  }
  {
    std::cout << "Loading hrp2" << std::endl;
    auto robots = mc_rbdyn::loadRobots({hrp2});
    std::cout << robots->robot().name() << std::endl;
    std::cout << "hrp2 OK\n" << std::endl;
  }
  {
    LOG_INFO("emplace_back modules");
    // std::vector<mc_rbdyn::RobotModule> modules;
    std::vector<mc_rbdyn::RobotModule, Eigen::aligned_allocator<mc_rbdyn::RobotModule>> modules;
    modules.emplace_back(*env);
    LOG_INFO("env emplaced");
    LOG_INFO(env->name);
    modules.emplace_back(*hrp2);
    LOG_INFO("hrp2 emplaced");
    modules.emplace_back(*jvrc);
    LOG_INFO("jvrc emplaced");
  }
  {
    std::cout << "Loading all" << std::endl;
    auto robots = mc_rbdyn::loadRobots({env, jvrc});
    //  std::cout << robots->robot().name() << std::endl;
    std::cout << "all OK" << std::endl;
  }
  {
    std::cout << "Loading all" << std::endl;
    auto robots = mc_rbdyn::loadRobots({env, jvrc, hrp2});
    //  std::cout << robots->robot().name() << std::endl;
    std::cout << "all OK" << std::endl;
  }
  return 0;
}
