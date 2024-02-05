#include <mc_rbdyn/gui/RobotConvex.h>

#include <mc_rtc/gui/Box.h>
#include <mc_rtc/gui/Cylinder.h>
#include <mc_rtc/gui/Polyhedron.h>
#include <mc_rtc/gui/Sphere.h>

#include <sch/S_Object/S_Box.h>
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Polyhedron/S_Polyhedron.h>

namespace mc_rbdyn::gui
{

void addConvexToGUI(mc_rtc::gui::StateBuilder & gui,
                    const std::vector<std::string> & category,
                    const mc_rbdyn::Robot & robot,
                    const std::string & name,
                    const mc_rtc::gui::PolyhedronConfig & cfg,
                    const std::optional<std::string> & publishName)
{
  const auto & convex = robot.convex(name);
  const auto & body = convex.first;
  const auto & c = convex.second;
  auto get_pose = [&, name, body]() { return robot.collisionTransform(name) * robot.bodyPosW(body); };
  auto publish_object = [&](auto && element) { gui.addElement(category, element); };
  if(auto * poly = dynamic_cast<sch::S_Polyhedron *>(c.get()))
  {
    const auto & sch_vertices = poly->getPolyhedronAlgorithm()->vertexes_;
    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(sch_vertices.size());
    for(const auto * v : sch_vertices)
    {
      const auto & c = v->getCoordinates();
      vertices.push_back({c.m_x, c.m_y, c.m_z});
    }
    auto tf_vertices = vertices;
    const auto & sch_triangles = poly->getPolyhedronAlgorithm()->triangles_;
    std::vector<std::array<size_t, 3>> triangles;
    triangles.reserve(sch_triangles.size());
    for(const auto & t : sch_triangles)
    {
      const auto a = sch_vertices[t.a]->getCoordinates();
      const auto b = sch_vertices[t.b]->getCoordinates();
      const auto c = sch_vertices[t.c]->getCoordinates();
      const auto normal = t.normal;
      auto cross = (a - b) ^ (a - c);
      auto dot = normal * cross;
      if(dot < 0) { triangles.push_back({t.c, t.b, t.a}); }
      else { triangles.push_back({t.a, t.b, t.c}); }
    }
    publish_object(mc_rtc::gui::Polyhedron(
        publishName.value_or(name), cfg,
        [vertices, tf_vertices, get_pose]() mutable -> const auto &
        {
          const auto & pose = get_pose();
          for(size_t i = 0; i < vertices.size(); ++i)
          {
            tf_vertices[i] = (sva::PTransformd{vertices[i]} * pose).translation();
          }
          return tf_vertices;
        },
        [triangles]() -> const auto & { return triangles; }));
  }
  else if(auto * box = dynamic_cast<sch::S_Box *>(c.get()))
  {
    publish_object(mc_rtc::gui::Box(
        publishName.value_or(name),
        [box]()
        {
          Eigen::Vector3d res;
          box->getBoxParameters(res.x(), res.y(), res.z());
          return res;
        },
        get_pose, mc_rtc::gui::Color::Green));
  }
  else if(auto * cylinder = dynamic_cast<sch::S_Cylinder *>(c.get()))
  {
    publish_object(mc_rtc::gui::Cylinder(
        publishName.value_or(name),
        [cylinder]() {
          return mc_rtc::gui::CylinderParameters{cylinder->getRadius(), (cylinder->getP2() - cylinder->getP1()).norm()};
        },
        get_pose, mc_rtc::gui::Color::Green));
  }
  else if(auto * sphere = dynamic_cast<sch::S_Sphere *>(c.get()))
  {
    publish_object(mc_rtc::gui::Sphere(
        publishName.value_or(name), [sphere]() { return sphere->getRadius(); }, get_pose, mc_rtc::gui::Color::Green));
  }
  else { mc_rtc::log::warning("{} in {} cannot be displayed", name, robot.name()); }
}

} // namespace mc_rbdyn::gui
