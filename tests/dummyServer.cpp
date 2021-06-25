/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/ControllerServer.h>

#include <mc_rtc/gui.h>

#include <SpaceVecAlg/Conversions.h>

#include <chrono>
#include <thread>

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

#include "utils.h"

void setColor(rbd::parsers::Visual & visual, const mc_rtc::gui::Color & color)
{
  rbd::parsers::Material mat;
  rbd::parsers::Material::Color col;
  col.r = color.r;
  col.g = color.g;
  col.b = color.b;
  col.a = color.a;
  mat.type = rbd::parsers::Material::Type::COLOR;
  mat.data = col;
  visual.material = mat;
}

rbd::parsers::Visual makeSphere(double radius)
{
  rbd::parsers::Geometry::Sphere s;
  s.radius = radius;
  rbd::parsers::Visual out;
  out.origin = sva::PTransformd::Identity();
  out.geometry.type = rbd::parsers::Geometry::Type::SPHERE;
  out.geometry.data = s;
  setColor(out, {1, 0, 0, 0.2});
  return out;
}

rbd::parsers::Visual makeCylinder(double radius, double length)
{
  rbd::parsers::Geometry::Cylinder c;
  c.length = length;
  c.radius = radius;
  rbd::parsers::Visual out;
  out.origin = sva::PTransformd::Identity();
  out.geometry.type = rbd::parsers::Geometry::Type::CYLINDER;
  out.geometry.data = c;
  setColor(out, {0, 1, 0, 0.2});
  return out;
}

rbd::parsers::Visual makeBox(const Eigen::Vector3d & dim)
{
  rbd::parsers::Geometry::Box b;
  b.size = dim;
  rbd::parsers::Visual out;
  out.origin = sva::PTransformd::Identity();
  out.geometry.type = rbd::parsers::Geometry::Type::BOX;
  out.geometry.data = b;
  setColor(out, {0, 0, 1, 0.2});
  return out;
}

sva::PTransformd lookAt(const Eigen::Vector3d & position, const Eigen::Vector3d & target, const Eigen::Vector3d & up)
{
  Eigen::Matrix3d R;
  R.col(2) = (target - position).normalized();
  R.col(0) = up.cross(R.col(2)).normalized();
  R.col(1) = R.col(2).cross(R.col(0));
  Eigen::Matrix4d view = Eigen::Matrix4d::Identity();
  view.topLeftCorner<3, 3>() = R;
  view.topRightCorner<3, 1>() = position;
  return sva::conversions::fromHomogeneous(view);
}

struct DummyProvider
{
  double value = 42.0;
  Eigen::Vector3d point = Eigen::Vector3d(0., 1., 2.);
};

struct FakeZMPGraph
{
  using Color = mc_rtc::gui::Color;
  using Point = std::array<double, 2>;
  using Points = std::vector<Point>;
  using PolygonDescription = mc_rtc::gui::plot::PolygonDescription;
  void update(double t_)
  {
    auto t = 0.5 * (t_ - t0);
    zmp_x += speed / 2;
    zmp_y = sin(M_PI * zmp_x);
    if(t < 0.2)
    {
      if(start_ds)
      {
        // Reset the style of both feet
        start_ds = false;
        feet.back() = makeFoot(rfoot_x, -0.5);
        feet[feet.size() - 2].outline(Color::Black).fill(Color(0, 0, 0, 0));
      }
      start_ss = true;
    }
    else if(t < 1.0)
    {
      if(start_ss)
      {
        // Left foot flying:
        //   - support foot gets red outline with blue filling
        //   - flying foot moves and gets grayed
        start_ss = false;
        feet.push_back(makeFoot(lfoot_x, 0.5, Color(0.5, 0.5, 0.5)));
        feet[feet.size() - 2].outline(Color::Red).fill(Color::Blue);
        if(start_walk)
        {
          speed = speed / 2;
        }
      }
      lfoot_x += speed;
      for(auto & points : feet.back().points())
      {
        points[0] += speed;
      }
      start_ds = true;
    }
    else if(t < 1.2)
    {
      if(start_ds)
      {
        // Reset the style of both feet
        if(start_walk)
        {
          speed = speed * 2;
          start_walk = false;
        }
        start_ds = false;
        feet.back() = makeFoot(lfoot_x, 0.5);
        feet[feet.size() - 2].outline(Color::Black).fill(Color(0, 0, 0, 0));
      }
      start_ss = true;
    }
    else if(t < 2)
    {
      if(start_ss)
      {
        // Left foot flying:
        //   - support foot gets red outline with blue filling
        //   - flying foot moves and gets grayed
        start_ss = false;
        feet.push_back(makeFoot(rfoot_x, -0.5, Color(0.5, 0.5, 0.5)));
        feet[feet.size() - 2].outline(Color::Red).fill(Color::Blue);
      }
      rfoot_x += speed;
      for(auto & points : feet.back().points())
      {
        points[0] += speed;
      }
      start_ds = true;
    }
    else
    {
      t0 = t_;
    }
  }
  void reset(double t)
  {
    t0 = t;
    zmp_x = 0;
    zmp_y = 0;
    lfoot_x = 0;
    rfoot_x = 0;
    feet = {makeFoot(0, 0.5), makeFoot(0, -0.5)};
    start_walk = true;
    start_ss = false;
    start_ds = false;
    speed = 0.05;
  }
  bool start_walk = true;
  bool start_ss = false;
  bool start_ds = false;
  double t0 = 0;
  double zmp_x = 0;
  double zmp_y = 0;
  double lfoot_x = 0;
  double rfoot_x = 0;
  double speed = 0.05;
  static PolygonDescription makeFoot(double x, double y, Color color = Color(0, 0, 0))
  {
    return {Points{Point{x - 0.15, y - 0.15}, Point{x - 0.15, y + 0.15}, Point{x + 0.15, y + 0.15},
                   Point{x + 0.15, y - 0.15}},
            color};
  }

  std::vector<PolygonDescription> feet = {makeFoot(0, 0.5), makeFoot(0, -0.5)};

  Color color() const
  {
    if(std::abs(zmp_y) > 0.9)
    {
      return Color::Red;
    }
    else
    {
      return Color::Black;
    }
  }
};

struct TestServer
{
  TestServer();

  void publish();

  template<typename T>
  void add_demo_plot(const std::string & name, T callback);

  void make_table(size_t s);

  void switch_visual(const std::string & visual);

  mc_control::ControllerServer server{1.0, 1.0, {"ipc:///tmp/mc_rtc_pub.ipc"}, {"ipc:///tmp/mc_rtc_rep.ipc"}};
  DummyProvider provider;
  mc_rtc::gui::StateBuilder builder;
  bool check_ = true;
  std::string string_ = "default";
  int int_ = 0;
  double d_ = 0.;
  double slide_ = 0.;
  Eigen::VectorXd v_{Eigen::VectorXd::Ones(6)};
  Eigen::Vector3d v3_{-1., -1., 1.};
  Eigen::Vector3d vInt_ = {0.2, 0.2, 0.};
  std::string combo_ = "b";
  std::string data_combo_;
  sva::PTransformd rotStatic_{sva::RotZ(-M_PI), Eigen::Vector3d(1., 1., 0.)};
  sva::PTransformd rotInteractive_{Eigen::Vector3d{0., 0., 1.}};
  sva::PTransformd static_{sva::RotZ(-M_PI), Eigen::Vector3d(1., 0., 0.)};
  sva::PTransformd interactive_{Eigen::Vector3d{0., 1., 0.}};
  Eigen::Vector3d xytheta_{0., 2., M_PI / 3};
  Eigen::VectorXd xythetaz_;
  std::vector<Eigen::Vector3d> polygon_;
  std::vector<Eigen::Vector3d> polygonColor_;
  std::vector<Eigen::Vector3d> polygonLineConfig_;
  std::vector<std::vector<Eigen::Vector3d>> polygons_;
  std::vector<std::vector<Eigen::Vector3d>> polygonsColor_;
  std::vector<std::vector<Eigen::Vector3d>> polygonsLineConfig_;
  Eigen::Vector3d arrow_start_{0.5, 0.5, 0.};
  Eigen::Vector3d arrow_end_{0.5, 1., -0.5};
  sva::ForceVecd force_{{0., 0., 0.}, {-50., 50., 100.}};
  double t_ = 0.0;
  std::vector<std::string> table_header;
  std::vector<std::string> table_format;
  std::vector<std::vector<double>> table_data;
  std::vector<std::string> static_table_header = {"A", "B", "C"};
  std::vector<std::tuple<std::string, double, int>> static_table_data = {
      std::make_tuple<std::string, double, int>("Hello", 4.2001, -4),
      std::make_tuple<std::string, double, int>("World", 0.2001, 4),
      std::make_tuple<std::string, double, int>("!", 0.0001, 42)};
  FakeZMPGraph graph_;
  std::vector<Eigen::Vector3d> trajectory_ = {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
                                              -Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitY()};
  std::vector<sva::PTransformd> poseTrajectory_ = {{sva::RotX<double>(0), {1, 1, 1}},
                                                   {sva::RotX<double>(M_PI / 2), {1, -1, 2}},
                                                   {sva::RotY<double>(-M_PI / 2) * sva::RotX<double>(M_PI), {1, 1, 3}}};
  rbd::parsers::Visual visual_;
  std::string visualChoice_ = "sphere";
  double sphereRadius_ = 1.0;
  double cylinderRadius_ = 1.0;
  double cylinderLength_ = 1.0;
  Eigen::Vector3d boxDim_ = Eigen::Vector3d::Ones();
  std::string robotVisual_ = "NECK_P_S";
  sva::PTransformd visualPos_ = sva::PTransformd::Identity();

  std::shared_ptr<mc_rbdyn::Robots> robots_;
};

TestServer::TestServer() : xythetaz_(4)
{
  xythetaz_ << 1., 2., M_PI / 5, 1;

  polygon_.push_back({1, 1, 0});
  polygon_.push_back({1, -1, 0});
  polygon_.push_back({-1, -1, 0});
  polygon_.push_back({-1, 1, 0});

  polygonColor_.push_back({1, 1, 1});
  polygonColor_.push_back({1, -1, 1});
  polygonColor_.push_back({-1, -1, 1});
  polygonColor_.push_back({-1, 1, 1});

  polygonLineConfig_.push_back({1, 1, 2});
  polygonLineConfig_.push_back({1, -1, 2});
  polygonLineConfig_.push_back({-1, -1, 2});
  polygonLineConfig_.push_back({-1, 1, 2});

  auto makeFoot = [](const sva::PTransformd & pose) {
    std::vector<Eigen::Vector3d> points;
    double width = 0.1;
    double length = 0.2;
    points.push_back((sva::PTransformd{Eigen::Vector3d{length / 2, width / 2, 0}} * pose).translation());
    points.push_back((sva::PTransformd{Eigen::Vector3d{length / 2, -width / 2, 0}} * pose).translation());
    points.push_back((sva::PTransformd{Eigen::Vector3d{-length / 2, -width / 2, 0}} * pose).translation());
    points.push_back((sva::PTransformd{Eigen::Vector3d{-length / 2, width / 2, 0}} * pose).translation());
    return points;
  };
  polygons_.push_back(makeFoot({Eigen::Vector3d(0, -0.15, 0)}));
  polygons_.push_back(makeFoot({Eigen::Vector3d(0, 0.15, 0)}));
  polygons_.push_back(makeFoot({Eigen::Vector3d(0.3, -0.15, 0)}));
  polygons_.push_back(makeFoot({Eigen::Vector3d(0.6, 0.15, 0)}));
  polygons_.push_back(makeFoot({Eigen::Vector3d(0.6, -0.15, 0)}));
  polygons_.push_back(makeFoot({Eigen::Vector3d(0.9, 0.15, 0.2)}));
  polygons_.push_back(makeFoot({Eigen::Vector3d(1.2, -0.15, 0.4)}));
  polygons_.push_back(makeFoot({Eigen::Vector3d(1.2, 0.15, 0.4)}));

  polygonsColor_.push_back(makeFoot({Eigen::Vector3d(0, 1 - 0.15, 0)}));
  polygonsColor_.push_back(makeFoot({Eigen::Vector3d(0, 1 + 0.15, 0)}));
  polygonsColor_.push_back(makeFoot({Eigen::Vector3d(0.3, 1 - 0.15, 0)}));
  polygonsColor_.push_back(makeFoot({Eigen::Vector3d(0.6, 1 + 0.15, 0)}));
  polygonsColor_.push_back(makeFoot({Eigen::Vector3d(0.6, 1 - 0.15, 0)}));
  polygonsColor_.push_back(makeFoot({Eigen::Vector3d(0.9, 1 + 0.15, 0.2)}));
  polygonsColor_.push_back(makeFoot({Eigen::Vector3d(1.2, 1 - 0.15, 0.4)}));
  polygonsColor_.push_back(makeFoot({Eigen::Vector3d(1.2, 1 + 0.15, 0.4)}));

  polygonsLineConfig_.push_back(makeFoot({Eigen::Vector3d(0, 2 - 0.15, 0)}));
  polygonsLineConfig_.push_back(makeFoot({Eigen::Vector3d(0, 2 + 0.15, 0)}));
  polygonsLineConfig_.push_back(makeFoot({Eigen::Vector3d(0.3, 2 - 0.15, 0)}));
  polygonsLineConfig_.push_back(makeFoot({Eigen::Vector3d(0.6, 2 + 0.15, 0)}));
  polygonsLineConfig_.push_back(makeFoot({Eigen::Vector3d(0.6, 2 - 0.15, 0)}));
  polygonsLineConfig_.push_back(makeFoot({Eigen::Vector3d(0.9, 2 + 0.15, 0.2)}));
  polygonsLineConfig_.push_back(makeFoot({Eigen::Vector3d(1.2, 2 - 0.15, 0.4)}));
  polygonsLineConfig_.push_back(makeFoot({Eigen::Vector3d(1.2, 2 + 0.15, 0.4)}));

  make_table(3);

  builder.addElement({"Table"},
                     mc_rtc::gui::Table("Static", static_table_header, [this]() { return static_table_data; }),
                     mc_rtc::gui::Table("Static formatted", static_table_header, {"s: {}", "d: {:0.2f}", "i: {:0.0f}"},
                                        [this]() { return static_table_data; }),
                     mc_rtc::gui::Table(
                         "Dynamic", [this]() { return table_header; }, [this]() { return table_data; }),
                     mc_rtc::gui::Table(
                         "Dynamic formatted", [this]() { return table_header; }, [this]() { return table_format; },
                         [this]() { return table_data; }),
                     mc_rtc::gui::Button("Add row", [this]() { make_table(table_data.size() + 1); }));
  auto data = builder.data();
  data.add("DataComboInput", std::vector<std::string>{"Choice A", "Choice B", "Choice C", "Obiwan Kenobi"});
  data.add("robots", std::vector<std::string>{"Goldorak", "Astro"});
  auto bodies = data.add("bodies");
  bodies.add("Goldorak", std::vector<std::string>{"FulguroPoing", "FulguroPied"});
  bodies.add("Astro", std::vector<std::string>{"Head", "Arm", "Foot"});
  auto surfaces = data.add("surfaces");
  surfaces.add("Goldorak", std::vector<std::string>{"Goldo_S1", "Goldo_S2", "Goldo_S3"});
  surfaces.add("Astro", std::vector<std::string>{"Astro_S1", "Astro_S2", "Astro_S3"});
  auto joints = data.add("joints");
  joints.add("Goldorak", std::vector<std::string>{"Goldo_J1", "Goldo_J2", "Goldo_J3"});
  joints.add("Astro", std::vector<std::string>{"Astro_J1", "Astro_J2", "Astro_J3"});
  builder.addElement({}, mc_rtc::gui::Label("Test", []() { return 2; }));
  builder.addElement({"Test"}, mc_rtc::gui::Button("Test", []() { std::cout << "Test::Test clicked!\n"; }));
  builder.addElement({"Test", "data"},
                     mc_rtc::gui::Button("Test", []() { std::cout << "Test::Test::Test clicked!\n"; }));
  builder.addElement(
      {"dummy", "provider"}, mc_rtc::gui::Label("value", [this]() { return provider.value; }),
      mc_rtc::gui::ArrayLabel("point", [this]() { return provider.point; }),
      mc_rtc::gui::ArrayLabel("point with labels", {"x", "y", "z"}, [this]() { return provider.point; }));
  builder.addElement({"Button example"}, mc_rtc::gui::Button("Push me", []() { mc_rtc::log::info("Button pushed"); }));
  builder.addElement({"Stacked buttons"}, mc_rtc::gui::ElementsStacking::Horizontal,
                     mc_rtc::gui::Button("Foo", []() { mc_rtc::log::info("Foo pushed"); }),
                     mc_rtc::gui::Button("Bar", []() { mc_rtc::log::info("Bar pushed"); }));
  builder.addElement({"Checkbox example"},
                     mc_rtc::gui::Checkbox(
                         "Checkbox", [this]() { return check_; }, [this]() { check_ = !check_; }));
  builder.addElement({"StringInput example"}, mc_rtc::gui::StringInput(
                                                  "StringInput", [this]() { return string_; },
                                                  [this](const std::string & data) {
                                                    string_ = data;
                                                    std::cout << "string_ changed to " << string_ << std::endl;
                                                  }));
  builder.addElement({"IntegerInput example"}, mc_rtc::gui::IntegerInput(
                                                   "IntegerInput", [this]() { return int_; },
                                                   [this](int data) {
                                                     int_ = data;
                                                     std::cout << "int_ changed to " << int_ << std::endl;
                                                   }));
  builder.addElement({"NumberInput example"}, mc_rtc::gui::NumberInput(
                                                  "NumberInput", [this]() { return d_; },
                                                  [this](double data) {
                                                    d_ = data;
                                                    std::cout << "d_ changed to " << d_ << std::endl;
                                                  }));
  builder.addElement({"NumberSlider example"}, mc_rtc::gui::NumberSlider(
                                                   "NumberSlider", [this]() { return slide_; },
                                                   [this](double s) {
                                                     slide_ = s;
                                                     std::cout << "slide_ changed to " << slide_ << std::endl;
                                                   },
                                                   -100.0, 100.0));
  builder.addElement({"ArrayInput example"}, mc_rtc::gui::ArrayInput(
                                                 "ArrayInput", [this]() { return v_; },
                                                 [this](const Eigen::VectorXd & data) {
                                                   v_ = data;
                                                   std::cout << "v_ changed to " << v_.transpose() << std::endl;
                                                 }));
  builder.addElement({"ArrayInput with labels example"},
                     mc_rtc::gui::ArrayInput(
                         "ArrayInput with labels", {"x", "y", "z"}, [this]() { return v3_; },
                         [this](const Eigen::Vector3d & data) {
                           v3_ = data;
                           std::cout << "v3_ changed to " << v3_.transpose() << std::endl;
                         }));
  builder.addElement({"ComboInput"}, mc_rtc::gui::ComboInput(
                                         "ComboInput", {"a", "b", "c", "d"}, [this]() { return combo_; },
                                         [this](const std::string & s) {
                                           combo_ = s;
                                           std::cout << "combo_ changed to " << combo_ << std::endl;
                                         }));
  builder.addElement({"DataComboInput"}, mc_rtc::gui::DataComboInput(
                                             "DataComboInput", {"DataComboInput"}, [this]() { return data_combo_; },
                                             [this](const std::string & s) {
                                               data_combo_ = s;
                                               std::cout << "data_combo_ changed to " << data_combo_ << std::endl;
                                             }));
  builder.addElement({"Schema"}, mc_rtc::gui::Schema("Add metatask", "MetaTask", [](const mc_rtc::Configuration & c) {
                       std::cout << "Got schema request:\n" << c.dump(true) << std::endl;
                     }));
  builder.addElement({"Contacts", "Add"},
                     mc_rtc::gui::Form(
                         "Add contact",
                         [](const mc_rtc::Configuration & data) {
                           std::cout << "Got data" << std::endl << data.dump(true) << std::endl;
                         },
                         mc_rtc::gui::FormCheckbox{"Enabled", false, true},
                         mc_rtc::gui::FormIntegerInput{"INT", false, 42},
                         mc_rtc::gui::FormNumberInput{"NUMBER", false, 0.42},
                         mc_rtc::gui::FormStringInput{"STRING", false, "a certain string"},
                         mc_rtc::gui::FormArrayInput<Eigen::Vector3d>{"ARRAY_FIXED_SIZE", false, {1, 2, 3}},
                         mc_rtc::gui::FormArrayInput<std::vector<double>>{"ARRAY_UNBOUNDED", false},
                         mc_rtc::gui::FormComboInput{"CHOOSE WISELY", false, {"A", "B", "C", "D"}},
                         mc_rtc::gui::FormDataComboInput{"R0", false, {"robots"}},
                         mc_rtc::gui::FormDataComboInput{"R0 surface", false, {"surfaces", "$R0"}},
                         mc_rtc::gui::FormDataComboInput{"R1", false, {"robots"}},
                         mc_rtc::gui::FormDataComboInput{"R1 surface", false, {"surfaces", "$R1"}}));
  builder.addElement(
      {"GUI Markers", "Transforms"}, mc_rtc::gui::Transform("ReadOnly Transform", [this]() { return static_; }),
      mc_rtc::gui::Transform(
          "Interactive Transform", [this]() { return interactive_; },
          [this](const sva::PTransformd & p) { interactive_ = p; }),
      mc_rtc::gui::XYTheta("XYTheta ReadOnly",
                           [this]() -> std::array<double, 4> {
                             return {xytheta_.x(), xytheta_.y(), xytheta_.z(), 0.1};
                           }),
      mc_rtc::gui::XYTheta(
          "XYTheta", [this]() { return xytheta_; }, [this](const Eigen::VectorXd & vec) { xytheta_ = vec.head<3>(); }),
      mc_rtc::gui::XYTheta(
          "XYThetaAltitude", [this]() { return xythetaz_; }, [this](const Eigen::VectorXd & vec) { xythetaz_ = vec; }),
      mc_rtc::gui::Rotation("ReadOnly Rotation", [this]() { return rotStatic_; }),
      mc_rtc::gui::Rotation(
          "Interactive Rotation", [this]() { return rotInteractive_; },
          [this](const Eigen::Quaterniond & q) { rotInteractive_.rotation() = q; }));

  builder.addElement(
      {"GUI Markers", "Point3D"},
      mc_rtc::gui::Point3D("ReadOnly", mc_rtc::gui::PointConfig({1., 0., 0.}, 0.08), [this]() { return v3_; }),
      mc_rtc::gui::Point3D(
          "Interactive", mc_rtc::gui::PointConfig({0., 1., 0.}, 0.08), [this]() { return vInt_; },
          [this](const Eigen::Vector3d & v) { vInt_ = v; }));

  auto orange = mc_rtc::gui::Color(1.0, 0.5, 0.0);
  auto pstyle = mc_rtc::gui::LineConfig(mc_rtc::gui::Color::Cyan, 0.1, mc_rtc::gui::LineStyle::Dotted);
  builder.addElement({"GUI Markers", "Polygons"}, mc_rtc::gui::Polygon("Single polygon", [this]() { return polygon_; }),
                     mc_rtc::gui::Polygon("Single polygon (color)", orange, [this]() { return polygonColor_; }),
                     mc_rtc::gui::Polygon("Single polygon (config)", pstyle, [this]() { return polygonLineConfig_; }),
                     mc_rtc::gui::Polygon("Polygons", [this]() { return polygons_; }),
                     mc_rtc::gui::Polygon("Polygons (color)", orange, [this]() { return polygonsColor_; }),
                     mc_rtc::gui::Polygon("Polygons (config)", pstyle, [this]() { return polygonsLineConfig_; }));

  builder.addElement(
      {"GUI Markers", "Trajectories"}, mc_rtc::gui::Trajectory("Vector3d", [this]() { return trajectory_; }),
      mc_rtc::gui::Trajectory("PTransformd", {mc_rtc::gui::Color::Green}, [this]() { return poseTrajectory_; }),
      mc_rtc::gui::Trajectory("Live 3D", {mc_rtc::gui::Color::Magenta},
                              [this]() {
                                return Eigen::Vector3d{cos(t_), sin(t_), 1.0};
                              }),
      mc_rtc::gui::Trajectory("Live transform", {mc_rtc::gui::Color::Black}, [this]() {
        return lookAt({cos(t_), -1, sin(t_)}, {0, 0, 0}, Eigen::Vector3d::UnitZ());
      }));

  mc_rtc::gui::ArrowConfig arrow_config({1., 0., 0.});
  arrow_config.start_point_scale = 0.02;
  arrow_config.end_point_scale = 0.02;
  builder.addElement({"GUI Markers", "Arrows"},
                     mc_rtc::gui::Arrow(
                         "ArrowRO", arrow_config,
                         []() {
                           return Eigen::Vector3d{2, 2, 0};
                         },
                         []() {
                           return Eigen::Vector3d{2.5, 2.5, 0.5};
                         }),
                     mc_rtc::gui::Arrow(
                         "Arrow", arrow_config, [this]() { return arrow_start_; },
                         [this](const Eigen::Vector3d & start) { arrow_start_ = start; },
                         [this]() { return arrow_end_; }, [this](const Eigen::Vector3d & end) { arrow_end_ = end; }),
                     mc_rtc::gui::Force(
                         "ForceRO", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0., 0.)),
                         []() {
                           return sva::ForceVecd(Eigen::Vector3d{0., 0., 0.}, Eigen::Vector3d{10., 0., 100.});
                         },
                         []() {
                           return sva::PTransformd{Eigen::Vector3d{2, 2, 0}};
                         }),
                     mc_rtc::gui::Force(
                         "Force", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(0., 1., 0.)), [this]() { return force_; },
                         [this](const sva::ForceVecd & force) { force_ = force; },
                         []() {
                           return sva::PTransformd{Eigen::Vector3d{2, 2, 0}};
                         }));
  using Color = mc_rtc::gui::Color;
  using Range = mc_rtc::gui::plot::Range;
  using Style = mc_rtc::gui::plot::Style;
  using Side = mc_rtc::gui::plot::Side;
  auto sin_cos_plot = [this](const std::string & name) {
    builder.addPlot(name, mc_rtc::gui::plot::X("t", [this]() { return t_; }),
                    mc_rtc::gui::plot::Y(
                        "sin(t)", [this]() { return std::sin(t_); }, Color::Red),
                    mc_rtc::gui::plot::Y(
                        "cos(t)", [this]() { return std::cos(t_); }, Color::Blue));
  };
  add_demo_plot("sin(t)/cos(t)", sin_cos_plot);
  auto demo_style_plot = [this](const std::string & name) {
    builder.addPlot(name, mc_rtc::gui::plot::X("t", [this]() { return t_; }),
                    mc_rtc::gui::plot::Y(
                        "Solid", [this]() { return std::cos(t_); }, Color::Red)
                        .style(Style::Solid),
                    mc_rtc::gui::plot::Y(
                        "Dashed", [this]() { return 2 - std::cos(t_); }, Color::Blue, Style::Dashed),
                    mc_rtc::gui::plot::Y(
                        "Dotted", [this]() { return std::sin(t_); }, Color::Green)
                        .style(Style::Dotted)
                        .side(Side::Right),
                    mc_rtc::gui::plot::Y(
                        "Point", [this]() { return 2 - std::sin(t_); }, Color::Magenta, Style::Point, Side::Right));
  };
  add_demo_plot("Demo style", demo_style_plot);
  auto fix_axis_plot = [this](const std::string & name) {
    builder.addPlot(name, mc_rtc::gui::plot::X("t", [this]() { return t_; }), {"Y1", {0, 1}}, // Fix both min and max
                    {"Y2", {-Range::inf, 0}}, // Only fix max
                    mc_rtc::gui::plot::Y(
                        "sin(t)", [this]() { return std::sin(t_); }, Color::Red),
                    mc_rtc::gui::plot::Y(
                        "cos(t)", [this]() { return std::cos(t_); }, Color::Blue, Style::Solid, Side::Right));
  };
  add_demo_plot("Fix axis", fix_axis_plot);
  using PolygonDescription = mc_rtc::gui::plot::PolygonDescription;
  using Point = std::array<double, 2>;
  using Points = std::vector<Point>;
  auto circle_plot = [this](const std::string & name) {
    builder.addXYPlot(name, {"X (m)", {-1, 1}}, {"Y (m)", {-1, 1}},
                      mc_rtc::gui::plot::XY(
                          "Round", [this]() { return std::cos(t_); }, [this]() { return std::sin(t_); }, Color::Red),
                      mc_rtc::gui::plot::Polygon("Square", []() {
                        return PolygonDescription(Points{Point{-1, -1}, Point{-1, 1}, Point{1, 1}, Point{1, -1}},
                                                  Color::Blue)
                            .fill(Color(0, 1, 0, 0.75));
                      }));
  };
  add_demo_plot("Circle in square", circle_plot);
  auto redSquareBlueFill =
      PolygonDescription(Points{Point{-1, -1}, Point{-1, 1}, Point{1, 1}, Point{1, -1}}, Color::Red).fill(Color::Blue);
  auto purpleTriangleYellowFill =
      PolygonDescription(Points{Point{1, 0}, Point{1.5, 2}, Point{2, -2}}, Color::Magenta).fill(Color::Yellow);
  auto blueRectangle = PolygonDescription(Points{Point{-2, -2}, Point{2, -2}, Point{2, -3}, Point{-2, -3}}, Color::Blue)
                           .style(Style::Dotted);
  std::vector<PolygonDescription> polygons = {redSquareBlueFill, purpleTriangleYellowFill, blueRectangle};
  auto polygons_plot = [this, polygons](const std::string & name) {
    builder.addXYPlot(name, mc_rtc::gui::plot::Polygons("Polygons", [polygons]() { return polygons; }));
  };
  add_demo_plot("Polygons demo", polygons_plot);
  auto fake_zmp_plot = [this](const std::string & name) {
    graph_.reset(t_);
    builder.addXYPlot(name,
                      mc_rtc::gui::plot::XY(
                          "ZMP", [this]() { return graph_.zmp_x; }, [this] { return graph_.zmp_y; },
                          [this]() { return graph_.color(); }),
                      mc_rtc::gui::plot::Polygons("feet", [this]() { return graph_.feet; }));
  };
  add_demo_plot("Fake ZMP", fake_zmp_plot);

  switch_visual("sphere");

  configureRobotLoader();
  robots_ = mc_rbdyn::loadRobot(*mc_rbdyn::RobotLoader::get_robot_module("JVRC1"));
  auto & robot = robots_->robot("jvrc1");
  robot.posW({Eigen::Vector3d{-2.0, 0.0, robot.posW().translation().z()}});
  builder.addElement(
      {"Robot"}, mc_rtc::gui::Robot("jvrc1", [this]() -> const mc_rbdyn::Robot & { return robots_->robot("jvrc1"); }));
}

void TestServer::switch_visual(const std::string & choice)
{
  visualChoice_ = choice;
  builder.removeCategory({"Visual"});
  builder.addElement({"Visual"}, mc_rtc::gui::ComboInput(
                                     "Choice", {"sphere", "box", "cylinder", "mesh"},
                                     [this]() -> const std::string & { return visualChoice_; },
                                     [this](const std::string & c) { switch_visual(c); }));
  if(choice == "sphere")
  {
    visual_ = makeSphere(sphereRadius_);
    builder.addElement({"Visual"}, mc_rtc::gui::NumberInput(
                                       "radius", [this]() { return sphereRadius_; },
                                       [this](double r) {
                                         auto & v = boost::get<rbd::parsers::Geometry::Sphere>(visual_.geometry.data);
                                         v.radius = r;
                                         sphereRadius_ = r;
                                       }));
  }
  else if(choice == "box")
  {
    visual_ = makeBox(boxDim_);
    builder.addElement({"Visual"},
                       mc_rtc::gui::ArrayInput(
                           "dimensions", {"x", "y", "z"}, [this]() -> const Eigen::Vector3d & { return boxDim_; },
                           [this](const Eigen::Vector3d & v) {
                             auto & b = boost::get<rbd::parsers::Geometry::Box>(visual_.geometry.data);
                             b.size = v;
                             boxDim_ = v;
                           }));
  }
  else if(choice == "cylinder")
  {
    visual_ = makeCylinder(cylinderRadius_, cylinderLength_);
    builder.addElement({"Visual"},
                       mc_rtc::gui::NumberInput(
                           "radius", [this]() { return cylinderRadius_; },
                           [this](double r) {
                             auto & c = boost::get<rbd::parsers::Geometry::Cylinder>(visual_.geometry.data);
                             c.radius = r;
                             cylinderRadius_ = r;
                           }),
                       mc_rtc::gui::NumberInput(
                           "length", [this]() { return cylinderLength_; },
                           [this](double r) {
                             auto & c = boost::get<rbd::parsers::Geometry::Cylinder>(visual_.geometry.data);
                             c.length = r;
                             cylinderLength_ = r;
                           }));
  }
  else if(choice == "mesh")
  {
    const auto & robot = robots_->robot("jvrc1");
    const auto & visuals = robot.module()._visual.at(robotVisual_);
    for(auto & v : visuals)
    {
      builder.addElement({"Visual"}, mc_rtc::gui::Visual(
                                         v.name, [&]() -> const rbd::parsers::Visual { return v; },
                                         [this]() -> const sva::PTransformd & { return visualPos_; }));
    }
    std::vector<std::string> choices;
    choices.reserve(robot.mb().bodies().size());
    for(const auto & b : robot.mb().bodies())
    {
      if(robot.module()._visual.count(b.name()))
      {
        choices.push_back(b.name());
      }
    }
    builder.addElement({"Visual"}, mc_rtc::gui::ComboInput(
                                       "body", choices, [this]() { return robotVisual_; },
                                       [this](const std::string & s) {
                                         robotVisual_ = s;
                                         switch_visual("mesh");
                                       }));
  }
  builder.addElement({"Visual", "Position"},
                     mc_rtc::gui::Transform(
                         "position", [this]() -> const sva::PTransformd & { return visualPos_; },
                         [this](const sva::PTransformd & p) { visualPos_ = p; }));
  if(choice == "mesh")
  {
    return;
  }
  builder.addElement({"Visual"}, mc_rtc::gui::Visual(
                                     choice, [this]() -> const rbd::parsers::Visual & { return visual_; },
                                     [this]() -> const sva::PTransformd & { return visualPos_; }));
}

template<typename T>
void TestServer::add_demo_plot(const std::string & name, T callback)
{
  bool has_plot = false;
  builder.addElement({}, mc_rtc::gui::Button("Add " + name + " plot", [has_plot, callback, name, this]() mutable {
                       if(has_plot)
                       {
                         has_plot = false;
                         builder.removePlot(name);
                       }
                       else
                       {
                         has_plot = true;
                         callback(name);
                       }
                     }));
}

void TestServer::publish()
{
  graph_.update(t_);
  server.handle_requests(builder);
  server.publish(builder);
  t_ += 0.05;
}

void TestServer::make_table(size_t s)
{
  std::vector<std::string> header;
  std::vector<std::string> format;
  std::vector<std::vector<double>> data;
  for(size_t i = 1; i <= s; ++i)
  {
    header.push_back(std::to_string(i));
    format.push_back("{:0." + std::to_string(i) + "f}");
  }
  double d = 1.1;
  for(size_t i = 1; i <= s; ++i)
  {
    data.emplace_back();
    auto & vec = data.back();
    for(size_t j = 1; j <= s; ++j)
    {
      vec.push_back(std::pow(d, j));
    }
    d += 1.1;
  }
  table_header = std::move(header);
  table_format = std::move(format);
  table_data = std::move(data);
}

int main()
{
  TestServer server;
  while(1)
  {
    server.publish();
    std::this_thread::sleep_for(std::chrono::microseconds(50000));
  }
  return 0;
}
