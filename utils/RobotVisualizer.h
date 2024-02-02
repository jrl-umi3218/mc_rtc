#include <mc_rtc/gui.h>

#include <mc_control/ControllerServer.h>
#include <mc_rbdyn/gui/RobotConvex.h>
#include <mc_rbdyn/gui/RobotSurface.h>

struct RobotVisualizer
{
  /** Constructor
   *
   * \param params Initial loading parameters
   *
   * \param show_convexes Show all convexes when starting the server
   *
   * \param show_surfaces Show all surfaces when starting the server
   */
  RobotVisualizer(const std::vector<std::string> & params, bool show_convexes = false, bool show_surfaces = false);

  /** Run the visualizer forever */
  void run();

private:
  bool show_convexes;
  bool show_surfaces;
  mc_control::ControllerServerConfiguration server_config;
  mc_control::ControllerServer server{0.005, server_config};
  mc_rtc::gui::StateBuilder builder;
  mc_rtc::gui::PolyhedronConfig convexConfig = mc_rbdyn::gui::defaultConvexConfig;
  mc_rtc::gui::LineConfig surfaceConfig = mc_rbdyn::gui::defaultSurfaceConfig;

  std::vector<std::string> available_robots;
  int selected_robot = -1;
  std::unordered_map<std::string, bool> selected_convexes;
  std::unordered_map<std::string, bool> selected_frames;
  std::unordered_map<std::string, bool> selected_surfaces;
  std::unordered_map<std::string, std::vector<std::string>> surfaces_elements;
  std::shared_ptr<mc_rbdyn::Robots> robots;

  void loadRobot(const std::vector<std::string> & params);

  void addRobot();

  void removeRobot();

  void addConvex(const std::string & name);

  void addConvexConfigurationGUI();

  void removeConvex(const std::string & name);

  void addSurface(const std::string & name);

  void addSurfaceConfigurationGUI();

  void removeSurface(const std::string & name);

  void addFrame(const std::string & name);

  void removeFrame(const std::string & name);

  void setupRobotSelector();

  inline bool all_surfaces_selected() const noexcept
  {
    return std::all_of(selected_surfaces.begin(), selected_surfaces.end(), [](const auto & it) { return it.second; });
  }

  inline bool all_convexes_selected() const noexcept
  {
    return std::all_of(selected_convexes.begin(), selected_convexes.end(), [](const auto & it) { return it.second; });
  }

  inline bool all_frames_selected() const noexcept
  {
    return std::all_of(selected_frames.begin(), selected_frames.end(), [](auto && it) { return it.second; });
  }
};

/** Utility function to get robot parameters from main arguments */
std::vector<std::string> params_from_main(int argc, char * argv[]);
