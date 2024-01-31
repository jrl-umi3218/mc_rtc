#include <mc_rtc/gui.h>

#include <mc_rtc/io_utils.h>

#include <mc_rbdyn/RobotLoader.h>

#include <mc_control/ControllerServer.h>

#include <mc_rbdyn/gui/RobotConvex.h>

#ifdef MC_RTC_HAS_ROS
#endif

#include <thread>

struct ApplicationServer
{
  ApplicationServer(const std::vector<std::string> & params)
  {
    server_config.print_serving_information();
    available_robots = mc_rbdyn::RobotLoader::available_robots();
    loadRobot(params);
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

  void loadRobot(const std::vector<std::string> & params)
  {
    selected_robot = -1;
    if(!params.empty())
    {
      auto selected_it = std::find(available_robots.begin(), available_robots.end(), params[0]);
      if(selected_it != available_robots.end())
      {
        selected_robot = static_cast<int>(std::distance(available_robots.begin(), selected_it));
      }
    }
    try
    {
      auto rm = mc_rbdyn::RobotLoader::get_robot_module(params);
      removeRobot();
      robots = mc_rbdyn::Robots::make();
      robots->load(*rm);
      addRobot();
    }
    catch(...)
    {
      mc_rtc::log::error("Failed to load [{}]", mc_rtc::io::to_string(params));
    }
    setupRobotSelector();
  }

  void addRobot()
  {
    const auto & robot = robots->robot();
    builder.addElement({"Robot"}, mc_rtc::gui::Robot(robot.name(), [&robot]() -> const auto & { return robot; }));
    for(const auto & convex_it : robot.convexes())
    {
      mc_rbdyn::gui::addConvexToGUI(builder, {"Robot", "Collision objects"}, robot, convex_it.first);
    }
  }

  void removeRobot() { builder.removeCategory({"Robot"}); }

  void setupRobotSelector()
  {
    if(builder.hasElement({}, "Switch robot")) { builder.removeElement({}, "Switch robot"); }
    builder.addElement({}, mc_rtc::gui::Form(
                               "Switch robot",
                               [this](const mc_rtc::Configuration & data)
                               { this->loadRobot({data("Robot").operator std::string()}); },
                               mc_rtc::gui::FormComboInput("Robot", true, available_robots, false, selected_robot)));
  }

  std::vector<std::string> available_robots;
  int selected_robot = -1;
  std::shared_ptr<mc_rbdyn::Robots> robots;
};

int main(int argc, char * argv[])
{
  std::vector<std::string> params;
  for(int i = 1; i < argc; ++i) { params.push_back(argv[i]); }
  if(params.empty()) { params.push_back("JVRC1"); }
  ApplicationServer server(params);
  server.loop();
  return 0;
}
