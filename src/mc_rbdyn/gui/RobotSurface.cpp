#include <mc_rbdyn/gui/RobotSurface.h>

#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/gui/Cylinder.h>
#include <mc_rtc/gui/Polygon.h>

namespace mc_rbdyn::gui
{

std::vector<std::string> addSurfaceToGUI(mc_rtc::gui::StateBuilder & gui,
                                         const std::vector<std::string> & category,
                                         const mc_rbdyn::Robot & robot,
                                         const std::string & name,
                                         const mc_rtc::gui::LineConfig & cfg,
                                         const std::optional<std::string> & publishName)
{
  std::vector<std::string> out;
  const auto & surface = robot.surface(name);
  auto get_pose = [&robot, name]() { return robot.surfacePose(name); };
  auto publish_surface = [&](auto && element)
  {
    out.push_back(element.name());
    gui.addElement(category, element);
  };
  if(surface.type() == "cylindrical")
  {
    const auto & cylinder = dynamic_cast<const mc_rbdyn::CylindricalSurface &>(surface);
    publish_surface(mc_rtc::gui::Cylinder(
        publishName.value_or(name), [&cylinder]()
        { return mc_rtc::gui::CylinderParameters{cylinder.radius(), cylinder.width()}; }, get_pose, cfg.color));
  }
  else if(surface.type() == "planar")
  {
    const auto & plan = dynamic_cast<const mc_rbdyn::PlanarSurface &>(surface);
    // Surface publication
    std::vector<Eigen::Vector3d> points;
    points.resize(plan.points().size());
    publish_surface(mc_rtc::gui::Polygon(
        publishName.value_or(name), cfg,
        [&plan, points, get_pose]() mutable -> const std::vector<Eigen::Vector3d> &
        {
          auto pose = get_pose();
          for(size_t i = 0; i < points.size(); ++i)
          {
            const auto & plan_point = plan.planarPoints()[i];
            points[i] =
                (sva::PTransformd{Eigen::Vector3d(plan_point.first, plan_point.second, 0.0)} * pose).translation();
          }
          return points;
        }));
    // Normal publication
    mc_rtc::gui::ArrowConfig arrow_cfg(mc_rtc::gui::Color::Blue);
    arrow_cfg.shaft_diam = 0.01;
    arrow_cfg.head_diam = 0.02;
    arrow_cfg.head_len = 0.1;
    publish_surface(mc_rtc::gui::Arrow(
        publishName.value_or(name) + "_normal", arrow_cfg, [get_pose]() { return get_pose().translation(); },
        [get_pose]() { return (sva::PTransformd(Eigen::Vector3d(0, 0, 0.2)) * get_pose()).translation(); }));
  }
  else if(surface.type() == "gripper")
  {
    const auto & gripper = dynamic_cast<const mc_rbdyn::GripperSurface &>(surface);
    size_t pid = 0;
    mc_rtc::gui::ArrowConfig arrow_cfg(mc_rtc::gui::Color::Blue);
    arrow_cfg.shaft_diam = 0.005;
    arrow_cfg.head_diam = 0.01;
    arrow_cfg.head_len = 0.025;
    for(const auto & p : gripper.pointsFromOrigin())
    {
      auto get_start = [get_pose, &p]() { return (p * get_pose()).translation(); };
      auto get_end = [get_start, &p]() -> Eigen::Vector3d { return get_start() + p.rotation().col(2) * 0.05; };
      publish_surface(
          mc_rtc::gui::Arrow(publishName.value_or(name) + "_" + std::to_string(pid++), arrow_cfg, get_start, get_end));
    }
  }
  else { mc_rtc::log::error("[addSurfaceToGUI] Cannot handle surface type {}", surface.type()); }
  return out;
}

} // namespace mc_rbdyn::gui
