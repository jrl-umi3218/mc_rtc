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
#ifdef MC_RTC_HAS_ROS
    if(robots) { mc_rtc::ROSBridge::update_robot_publisher("control", 0.005, robots->robot()); }
#endif
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
  catch(const std::exception & exc)
  {
    mc_rtc::log::error("Failed to load [{}]\n Exception:\n{}", mc_rtc::io::to_string(params), exc.what());
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
                             "Switch robot", [this](const mc_rtc::Configuration & data)
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
  builder.addElement({"Robot", "Frames"}, mc_rtc::gui::Checkbox(
                                              "Show all", [this]() { return all_frames_selected(); },
                                              [this, &robot]()
                                              {
                                                auto callback = all_frames_selected() ? &RobotVisualizer::removeFrame
                                                                                      : &RobotVisualizer::addFrame;
                                                for(const auto & name : robot.frames()) { (this->*callback)(name); }
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
  addConvexConfigurationGUI();
  auto frames = robot.frames();
  std::sort(frames.begin(), frames.end());
  for(const auto & name : frames)
  {
    selected_frames[name] = false;
    builder.addElement({"Robot", "Frames"}, mc_rtc::gui::Checkbox(
                                                name, [this, name]() { return selected_frames[name]; },
                                                [this, name]()
                                                {
                                                  if(selected_frames[name]) { addFrame(name); }
                                                  else { removeFrame(name); }
                                                }));
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
  addSurfaceConfigurationGUI();
}

void RobotVisualizer::addConvexConfigurationGUI()
{
  builder.removeElement({"Robot", "Convexes", "Convex Display Configuration"}, "Apply Convex Configuration");
  builder.addElement({"Robot", "Convexes", "Convex Display Configuration"},
                     mc_rtc::gui::Form(
                         "Apply Convex Configuration",
                         [this](const mc_rtc::Configuration & data)
                         {
                           convexConfig.triangle_color = data("Triangle Color");
                           convexConfig.edge_config.color = data("Edge Color");
                           convexConfig.edge_config.width = data("Edge Width");
                           convexConfig.show_edges = data("Show edges");
                           convexConfig.vertices_config.color = data("Vertex Color");
                           convexConfig.vertices_config.scale = data("Vertex Scale");
                           convexConfig.show_vertices = data("Show vertices");
                           if(data("Apply to all"))
                           {
                             for(const auto & [name, selected] : selected_convexes)
                             {
                               if(selected)
                               {
                                 removeConvex(name);
                                 addConvex(name);
                               }
                             }
                           }
                           builder.removeCategory({"Robot", "Convexes", "Configuration"});
                           addConvexConfigurationGUI();
                         },
                         mc_rtc::gui::FormArrayInput("Triangle Color", false, convexConfig.triangle_color),
                         mc_rtc::gui::FormArrayInput("Edge Color", false, convexConfig.edge_config.color),
                         mc_rtc::gui::FormNumberInput("Edge Width", false, convexConfig.edge_config.width),
                         mc_rtc::gui::FormCheckbox("Show edges", false, convexConfig.show_edges),
                         mc_rtc::gui::FormArrayInput("Vertex Color", false, convexConfig.vertices_config.color),
                         mc_rtc::gui::FormNumberInput("Vertex Scale", false, convexConfig.vertices_config.scale),
                         mc_rtc::gui::FormCheckbox("Show vertices", false, convexConfig.show_vertices),
                         mc_rtc::gui::FormCheckbox("Apply to all", false, true)));
}

void RobotVisualizer::addSurfaceConfigurationGUI()
{
  builder.removeElement({"Robot", "Surfaces", "Surface Display Configuration"}, "Apply Surface Configuration");
  builder.addElement({"Robot", "Surfaces", "Surface Display Configuration"},
                     mc_rtc::gui::Form(
                         "Apply Surface Configuration",
                         [this](const mc_rtc::Configuration & data)
                         {
                           surfaceConfig.color = data("Color");
                           surfaceConfig.width = data("Width");
                           if(data("Apply to all"))
                           {
                             for(const auto & [name, selected] : selected_surfaces)
                             {
                               if(selected)
                               {
                                 removeSurface(name);
                                 addSurface(name);
                               }
                             }
                           }
                           builder.removeCategory({"Robot", "Surfaces", "Configuration"});
                           addSurfaceConfigurationGUI();
                         },
                         mc_rtc::gui::FormArrayInput("Color", false, surfaceConfig.color),
                         mc_rtc::gui::FormNumberInput("Width", false, surfaceConfig.width),
                         mc_rtc::gui::FormCheckbox("Apply to all", false, true)));
}

void RobotVisualizer::removeRobot()
{
  selected_convexes.clear();
  selected_frames.clear();
  selected_surfaces.clear();
  surfaces_elements.clear();
  builder.removeCategory({"Robot"});
}

void RobotVisualizer::addSurface(const std::string & name)
{
  if(selected_surfaces[name]) { return; }
  selected_surfaces[name] = true;
  surfaces_elements[name] =
      mc_rbdyn::gui::addSurfaceToGUI(builder, {"Robot", "Surface objects"}, robots->robot(), name, surfaceConfig);
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
  mc_rbdyn::gui::addConvexToGUI(builder, {"Robot", "Collision objects"}, robots->robot(), name, convexConfig);
}

void RobotVisualizer::removeConvex(const std::string & name)
{
  if(!selected_convexes[name]) { return; }
  selected_convexes[name] = false;
  builder.removeElement({"Robot", "Collision objects"}, name);
}

void RobotVisualizer::addFrame(const std::string & name)
{
  if(selected_frames[name]) { return; }
  selected_frames[name] = true;
  builder.addElement({"Robot", "Frame objects"},
                     mc_rtc::gui::Transform(name, [this, name]() { return robots->robot().frame(name).position(); }));
}

void RobotVisualizer::removeFrame(const std::string & name)
{
  if(!selected_frames[name]) { return; }
  selected_frames[name] = false;
  builder.removeElement({"Robot", "Frame objects"}, name);
}
