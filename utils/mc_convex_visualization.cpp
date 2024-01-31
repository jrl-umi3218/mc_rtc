#include <mc_rtc/gui.h>

#include <mc_rbdyn/RobotLoader.h>

#include <mc_control/ControllerServer.h>

#include <mc_rbdyn/gui/RobotConvex.h>

#ifdef MC_RTC_HAS_ROS
#endif

#include <thread>

struct ApplicationServer
{
  ApplicationServer(const mc_rbdyn::Robot & robot)
  {
    server_config.print_serving_information();
    builder.addElement({"Robot"}, mc_rtc::gui::Robot(robot.name(), [&robot]() -> const auto & { return robot; }));
    for(const auto & convex_it : robot.convexes())
    {
      mc_rbdyn::gui::addConvexToGUI(builder, {"Robot", "Collision objects"}, robot, convex_it.first);
    }
  }

  virtual void publish()
  {
    server.handle_requests(builder);
    server.publish(builder);
  }

  mc_control::ControllerServerConfiguration server_config;
  mc_control::ControllerServer server{0.005, server_config};
  mc_rtc::gui::StateBuilder builder;

  void loop()
  {
    while(1)
    {
      auto now = std::chrono::high_resolution_clock::now();
      publish();
      std::this_thread::sleep_until(now + std::chrono::milliseconds(5));
    }
  }
};

int main(int argc, char * argv[])
{
  std::vector<std::string> params;
  for(int i = 1; i < argc; ++i) { params.push_back(argv[i]); }
  if(params.empty()) { params.push_back("JVRC1"); }
  auto rm = mc_rbdyn::RobotLoader::get_robot_module(params);
  auto robots = mc_rbdyn::Robots::make();
  robots->load(*rm);
  ApplicationServer server(robots->robot());
  server.loop();
  return 0;
}
