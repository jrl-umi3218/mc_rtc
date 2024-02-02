#include "RobotVisualizer.h"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/gui/RobotConvex.h>
#include <mc_rbdyn/gui/RobotSurface.h>

#include <mc_rtc/io_utils.h>

#include <thread>

#ifdef MC_RTC_HAS_ROS
#  include <mc_rtc_ros/ros.h>
#endif

std::vector<std::string> params_from_main(int argc, char * argv[])
{
  if(argc == 1) { return {"JVRC1"}; }
  std::vector<std::string> params;
  for(int i = 1; i < argc; ++i) { params.push_back(argv[i]); }
  return params;
}

RobotVisualizer::RobotVisualizer(const std::vector<std::string> & params, bool show_convexes, bool show_surfaces)
: show_convexes(show_convexes), show_surfaces(show_surfaces)
{
  server_config.print_serving_information();
  available_robots = mc_rbdyn::RobotLoader::available_robots();
  loadRobot(params);
}

void RobotVisualizer::run()
{
  while(1)
  {
    auto now = std::chrono::high_resolution_clock::now();
    if(robots) { mc_rtc::ROSBridge::update_robot_publisher("control", 0.005, robots->robot()); }
    server.handle_requests(builder);
    server.publish(builder);
    std::this_thread::sleep_until(now + std::chrono::milliseconds(5));
  }
}

void RobotVisualizer::loadRobot(const std::vector<std::string> & params)
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

void RobotVisualizer::setupRobotSelector()
{
  if(builder.hasElement({}, "Switch robot")) { builder.removeElement({}, "Switch robot"); }
  builder.addElement({}, mc_rtc::gui::Form(
                             "Switch robot",
                             [this](const mc_rtc::Configuration & data)
                             { this->loadRobot({data("Robot").operator std::string()}); },
                             mc_rtc::gui::FormComboInput("Robot", true, available_robots, false, selected_robot)));
}

void RobotVisualizer::addRobot()
{
  const auto & robot = robots->robot();
  builder.addElement({"Robot"}, mc_rtc::gui::Robot(robot.name(), [&robot]() -> const auto & { return robot; }));
  builder.addElement({"Robot", "Convexes"}, mc_rtc::gui::Checkbox(
                                                "Show all", [this]() { return all_convexes_selected(); },
                                                [this, &robot]()
                                                {
                                                  auto callback = all_convexes_selected()
                                                                      ? &RobotVisualizer::removeConvex
                                                                      : &RobotVisualizer::addConvex;
                                                  for(const auto & [name, _] : robot.convexes())
                                                  {
                                                    (this->*callback)(name);
                                                  }
                                                }));
  builder.addElement({"Robot", "Surfaces"}, mc_rtc::gui::Checkbox(
                                                "Show all", [this]() { return all_surfaces_selected(); },
                                                [this, &robot]()
                                                {
                                                  auto callback = all_surfaces_selected()
                                                                      ? &RobotVisualizer::removeSurface
                                                                      : &RobotVisualizer::addSurface;
                                                  for(const auto & name : robot.availableSurfaces())
                                                  {
                                                    (this->*callback)(name);
                                                  }
                                                }));
  for(const auto & convex_it : robot.convexes())
  {
    const auto & name = convex_it.first;
    selected_convexes[name] = false;
    builder.addElement({"Robot", "Convexes"}, mc_rtc::gui::Checkbox(
                                                  "Show " + name, [this, name]() { return selected_convexes[name]; },
                                                  [this, name]()
                                                  {
                                                    if(selected_convexes[name]) { removeConvex(name); }
                                                    else { addConvex(name); }
                                                  }));
    if(show_convexes) { addConvex(name); }
  }
  for(const auto & name : robot.availableSurfaces())
  {
    selected_surfaces[name] = false;
    builder.addElement({"Robot", "Surfaces"}, mc_rtc::gui::Checkbox(
                                                  "Show " + name, [this, name]() { return selected_surfaces[name]; },
                                                  [this, name]()
                                                  {
                                                    if(selected_surfaces[name]) { removeSurface(name); }
                                                    else { addSurface(name); }
                                                  }));
    if(show_surfaces) { addSurface(name); }
  }
}

void RobotVisualizer::removeRobot()
{
  selected_convexes.clear();
  selected_surfaces.clear();
  surfaces_elements.clear();
  builder.removeCategory({"Robot"});
}

void RobotVisualizer::addSurface(const std::string & name)
{
  if(selected_surfaces[name]) { return; }
  selected_surfaces[name] = true;
  surfaces_elements[name] =
      mc_rbdyn::gui::addSurfaceToGUI(builder, {"Robot", "Surface objects"}, robots->robot(), name);
}

void RobotVisualizer::removeSurface(const std::string & name)
{
  if(!selected_surfaces[name]) { return; }
  selected_surfaces[name] = false;
  for(const auto & e : surfaces_elements[name]) { builder.removeElement({"Robot", "Surface objects"}, e); }
}

void RobotVisualizer::addConvex(const std::string & name)
{
  if(selected_convexes[name]) { return; }
  selected_convexes[name] = true;
  mc_rbdyn::gui::addConvexToGUI(builder, {"Robot", "Collision objects"}, robots->robot(), name);
}

void RobotVisualizer::removeConvex(const std::string & name)
{
  if(!selected_convexes[name]) { return; }
  selected_convexes[name] = false;
  builder.removeElement({"Robot", "Collision objects"}, name);
}
