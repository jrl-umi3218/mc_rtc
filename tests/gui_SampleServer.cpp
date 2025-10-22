/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/utils/heatmap.h>
#include <mc_rtc/visual_utils.h>

#include <SpaceVecAlg/Conversions.h>

#include "gui_TestServer.h"
#include "utils.h"

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
  std::array<double, 3> array = {3., 4., 5.};
};

struct ChunkyXYData
{
  double t = 0;
  bool has_plot = false;
  bool add_data = false;
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
        if(start_walk) { speed = speed / 2; }
      }
      lfoot_x += speed;
      for(auto & points : feet.back().points()) { points[0] += speed; }
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
      for(auto & points : feet.back().points()) { points[0] += speed; }
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
    if(std::abs(zmp_y) > 0.9) { return Color::Red; }
    else
    {
      return Color::Black;
    }
  }
};

struct SampleServer : public TestServer
{
  SampleServer();

  void publish() override;

  template<typename T>
  void add_demo_plot(const std::string & name, T callback);

  template<typename T, typename U>
  void add_demo_plot(const std::string & name,
                     T callback,
                     const std::string & dynamic_label,
                     U dynamic_callback,
                     size_t & n_plots,
                     size_t mod);

  void make_table(size_t s);

  void switch_visual(const std::string & visual);

  DummyProvider provider;
  bool check_ = true;
  std::string string_ = "default";
  int int_ = 0;
  double d_ = 0.;
  double slide_ = 0.;
  Eigen::VectorXd v_{Eigen::VectorXd::Ones(6)};
  sva::MotionVecd vel_{{0, 0, 0}, {0, 0, 1}};
  sva::AdmittanceVecd admittance_vec_{{0.01, 0.01, 0.01}, {0.05, 0.05, 0.03}};
  Eigen::Matrix3d mat3_ = sva::RotZ(1.57);
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
  std::vector<std::string> table_header;
  std::vector<std::string> table_format;
  std::vector<std::vector<double>> table_data;
  std::vector<std::string> static_table_header = {"A", "B", "C"};
  std::vector<std::tuple<std::string, double, int>> static_table_data = {
      std::make_tuple<std::string, double, int>("Hello", 4.2001, -4),
      std::make_tuple<std::string, double, int>("World", 0.2001, 4),
      std::make_tuple<std::string, double, int>("!", 0.0001, 42)};
  FakeZMPGraph graph_;
  ChunkyXYData chunky_xy_data_;
  size_t n_dynamic_regular_plot = 0;
  size_t n_dynamic_xy_plot = 0;
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

  mc_rbdyn::RobotsPtr robots_;
};

SampleServer::SampleServer() : xythetaz_(4)
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

  auto makeFoot = [](const sva::PTransformd & pose)
  {
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
  auto frames = data.add("frames");
  frames.add("Goldorak", std::vector<std::string>{"Goldo_F1", "Goldo_F2", "Goldo_F3"});
  frames.add("Astro", std::vector<std::string>{"Astro_F1", "Astro_F2", "Astro_F3"});
  auto joints = data.add("joints");
  joints.add("Goldorak", std::vector<std::string>{"Goldo_J1", "Goldo_J2", "Goldo_J3"});
  joints.add("Astro", std::vector<std::string>{"Astro_J1", "Astro_J2", "Astro_J3"});
  builder.addElement(
      {"Labels"}, mc_rtc::gui::Label("value", [this]() { return provider.value; }),
      mc_rtc::gui::ArrayLabel("point", [this]() { return provider.point; }),
      mc_rtc::gui::ArrayLabel("point with labels", {"x", "y", "z"}, [this]() { return provider.point; }));
  builder.addElement({"Labels", "Simple syntax"}, mc_rtc::gui::Label("value", provider.value),
                     mc_rtc::gui::ArrayLabel("array", provider.array),
                     mc_rtc::gui::ArrayLabel("point w/ automatic labels", provider.point),
                     mc_rtc::gui::ArrayLabel("point w/ labels", {"1", "2", "3"}, provider.point),
                     mc_rtc::gui::RPYLabel("RPY [deg]", mat3_), mc_rtc::gui::RPYLabel<false>("RPY [rad]", mat3_));
  builder.addElement({"Buttons"}, mc_rtc::gui::Button("Push me", []() { mc_rtc::log::info("Button pushed"); }));
  builder.addElement({"Buttons"}, mc_rtc::gui::ElementsStacking::Horizontal,
                     mc_rtc::gui::Button("Foo", []() { mc_rtc::log::info("Foo pushed"); }),
                     mc_rtc::gui::Button("Bar", []() { mc_rtc::log::info("Bar pushed"); }));
  builder.addElement({"Checkbox"},
                     mc_rtc::gui::Checkbox("Checkbox", [this]() { return check_; }, [this]() { check_ = !check_; }));
  builder.addElement({"Checkbox", "Simple syntax"}, mc_rtc::gui::Checkbox("Checkbox", check_));
  builder.addElement({"Inputs"},
                     mc_rtc::gui::StringInput(
                         "StringInput", [this]() { return string_; },
                         [this](const std::string & data)
                         {
                           string_ = data;
                           std::cout << "string_ changed to " << string_ << std::endl;
                         }),
                     mc_rtc::gui::IntegerInput(
                         "IntegerInput", [this]() { return int_; },
                         [this](int data)
                         {
                           int_ = data;
                           std::cout << "int_ changed to " << int_ << std::endl;
                         }),
                     mc_rtc::gui::NumberInput(
                         "NumberInput", [this]() { return d_; },
                         [this](double data)
                         {
                           d_ = data;
                           std::cout << "d_ changed to " << d_ << std::endl;
                         }),
                     mc_rtc::gui::NumberSlider(
                         "NumberSlider", [this]() { return slide_; },
                         [this](double s)
                         {
                           slide_ = s;
                           std::cout << "slide_ changed to " << slide_ << std::endl;
                         },
                         -100.0, 100.0),
                     mc_rtc::gui::ArrayInput(
                         "ArrayInput", [this]() { return v_; },
                         [this](const Eigen::VectorXd & data)
                         {
                           v_ = data;
                           std::cout << "v_ changed to " << v_.transpose() << std::endl;
                         }),
                     mc_rtc::gui::ArrayInput(
                         "ArrayInput with labels", {"x", "y", "z"}, [this]() { return v3_; },
                         [this](const Eigen::Vector3d & data)
                         {
                           v3_ = data;
                           std::cout << "v3_ changed to " << v3_.transpose() << std::endl;
                         }),
                     mc_rtc::gui::ComboInput(
                         "ComboInput", {"a", "b", "c", "d"}, [this]() { return combo_; },
                         [this](const std::string & s)
                         {
                           combo_ = s;
                           std::cout << "combo_ changed to " << combo_ << std::endl;
                         }),
                     mc_rtc::gui::DataComboInput(
                         "DataComboInput", {"DataComboInput"}, [this]() { return data_combo_; },
                         [this](const std::string & s)
                         {
                           data_combo_ = s;
                           std::cout << "data_combo_ changed to " << data_combo_ << std::endl;
                         }));
  builder.addElement(
      {"Inputs", "Simple syntax"}, mc_rtc::gui::StringInput("StringInput", string_),
      mc_rtc::gui::IntegerInput("IntegerInput", int_), mc_rtc::gui::NumberInput("NumberInput", d_),
      mc_rtc::gui::NumberSlider("NumberSlider", slide_, -100.0, 100.0), mc_rtc::gui::ArrayInput("ArrayInput", v_),
      mc_rtc::gui::ArrayInput("ArrayInput w/ auto labels", v3_),
      mc_rtc::gui::ArrayInput("ArrayInput w/ labels", {"1", "2", "3"}, v3_),
      mc_rtc::gui::ComboInput("ComboInput", {"a", "b", "c", "d"}, combo_),
      mc_rtc::gui::DataComboInput("DataComboInput", {"DataComboInput"}, data_combo_),
      mc_rtc::gui::RPYInput("Rotation RPY [deg]", mat3_), mc_rtc::gui::RPYInput<false>("Rotation RPY [rad]", mat3_));
  builder.addElement({"Inputs", "Generic"}, mc_rtc::gui::Input("StringInput", string_),
                     mc_rtc::gui::Input("IntegerInput", int_), mc_rtc::gui::Input("NumberInput", d_),
                     mc_rtc::gui::Input("ArrayInput", v_), mc_rtc::gui::Input("ArrayInput w/ auto labels", v3_));
  builder.addElement({"Schema"},
                     mc_rtc::gui::Schema("Add metatask", "MetaTask", [](const mc_rtc::Configuration & c)
                                         { std::cout << "Got schema request:\n"
                                                     << c.dump(true) << std::endl; }));
  builder.addElement({"Forms", "Static"},
                     mc_rtc::gui::Form(
                         "Add contact", [](const mc_rtc::Configuration & data)
                         { std::cout << "Got data" << std::endl
                                     << data.dump(true) << std::endl; },
                         mc_rtc::gui::FormCheckbox("Enabled", false, true),
                         mc_rtc::gui::FormIntegerInput("INT", false, 42),
                         mc_rtc::gui::FormNumberInput("NUMBER", false, 0.42),
                         mc_rtc::gui::FormStringInput("STRING", false, "a certain string"),
                         mc_rtc::gui::FormArrayInput<Eigen::Vector3d>("ARRAY_FIXED_SIZE", false, {1, 2, 3}),
                         mc_rtc::gui::FormArrayInput<std::vector<double>>("ARRAY_UNBOUNDED", false),
                         mc_rtc::gui::FormComboInput{"CHOOSE WISELY", false, {"A", "B", "C", "D"}},
                         mc_rtc::gui::FormDataComboInput{"R0", false, {"robots"}},
                         mc_rtc::gui::FormDataComboInput{"R0 surface", false, {"surfaces", "$R0"}},
                         mc_rtc::gui::FormDataComboInput{"R1", false, {"robots"}},
                         mc_rtc::gui::FormDataComboInput{"R1 surface", false, {"surfaces", "$R1"}}));
  static double start_t = 0.0;
  static Eigen::Vector3d start_v = Eigen::Vector3d::Zero();
  builder.addElement({"Forms", "Dynamic"},
                     mc_rtc::gui::Form(
                         "Restart at given values",
                         [](const mc_rtc::Configuration & data)
                         {
                           auto prev = start_t;
                           start_t = data("Start");
                           mc_rtc::log::info("start_t was {:.2f} and is now {:.2f}", prev, start_t);
                           auto prev_v = start_v;
                           start_v = data("StartVector");
                           mc_rtc::log::info("start_v was {} and is now {}", MC_FMT_STREAMED(prev_v.transpose()),
                                             MC_FMT_STREAMED(start_v.transpose()));
                         },
                         mc_rtc::gui::FormNumberInput("Start", true,
                                                      []()
                                                      {
                                                        start_t += 0.05;
                                                        return start_t;
                                                      }),
                         mc_rtc::gui::FormArrayInput(
                             "StartVector", true,
                             []() -> const Eigen::Vector3d &
                             {
                               start_v += Eigen::Vector3d{0.5, 1.0, 2.0};
                               return start_v;
                             },
                             true)));
  builder.addElement(
      {"GUI Markers", "Transforms"}, mc_rtc::gui::Transform("ReadOnly Transform", [this]() { return static_; }),
      mc_rtc::gui::Transform(
          "Interactive Transform", [this]() { return interactive_; },
          [this](const sva::PTransformd & p) { interactive_ = p; }),
      mc_rtc::gui::XYTheta("XYTheta ReadOnly", [this]() -> std::array<double, 4>
                           { return {xytheta_.x(), xytheta_.y(), xytheta_.z(), 0.1}; }),
      mc_rtc::gui::XYTheta(
          "XYTheta", [this]() { return xytheta_; }, [this](const Eigen::VectorXd & vec) { xytheta_ = vec.head<3>(); }),
      mc_rtc::gui::XYTheta(
          "XYThetaAltitude", [this]() { return xythetaz_; }, [this](const Eigen::VectorXd & vec) { xythetaz_ = vec; }),
      mc_rtc::gui::Rotation("ReadOnly Rotation", [this]() { return rotStatic_; }),
      mc_rtc::gui::Rotation(
          "Interactive Rotation", [this]() { return rotInteractive_; },
          [this](const Eigen::Quaterniond & q) { rotInteractive_.rotation() = q; }));

  builder.addElement({"GUI Markers", "Transforms", "Simple syntax"},
                     mc_rtc::gui::TransformRO("ReadOnly Transform", static_),
                     mc_rtc::gui::Transform("Interactive Transform", interactive_));

  builder.addElement(
      {"GUI Markers", "Point3D"},
      mc_rtc::gui::Point3D("ReadOnly", mc_rtc::gui::PointConfig({1., 0., 0.}, 0.08), [this]() { return v3_; }),
      mc_rtc::gui::Point3D(
          "Interactive", mc_rtc::gui::PointConfig({0., 1., 0.}, 0.08), [this]() { return vInt_; },
          [this](const Eigen::Vector3d & v) { vInt_ = v; }));

  builder.addElement({"GUI Markers", "Point3D", "Simple syntax"},
                     mc_rtc::gui::Point3DRO("ReadOnly", const_cast<const Eigen::Vector3d &>(v3_)),
                     mc_rtc::gui::Point3D("Interactive", vInt_));

  auto orange = mc_rtc::gui::Color(1.0, 0.5, 0.0);
  auto pstyle = mc_rtc::gui::LineConfig(mc_rtc::gui::Color::Cyan, 0.1, mc_rtc::gui::LineStyle::Dotted);
  builder.addElement({"GUI Markers", "Polygons"}, mc_rtc::gui::Polygon("Single polygon", [this]() { return polygon_; }),
                     mc_rtc::gui::Polygon("Single polygon (color)", orange, [this]() { return polygonColor_; }),
                     mc_rtc::gui::Polygon("Single polygon (config)", pstyle, [this]() { return polygonLineConfig_; }),
                     mc_rtc::gui::Polygon("Polygons", [this]() { return polygons_; }),
                     mc_rtc::gui::Polygon("Polygons (color)", orange, [this]() { return polygonsColor_; }),
                     mc_rtc::gui::Polygon("Polygons (config)", pstyle, [this]() { return polygonsLineConfig_; }));

  auto polyhedron_triangles_fn = [this]()
  {
    double z = std::max(0.1, (1 + cos(t_ / 2)) / 2);
    // clang-format off
    return std::vector<std::array<Eigen::Vector3d, 3>>
    {
      // Upper pyramid
      {{ {-1, -1, 0}, {1, -1, 0}, {0, 0, z} }},
      {{ {1, -1, 0}, {1, 1, 0}, {0, 0, z} }},
      {{ {1, 1, 0}, {-1, 1, 0}, {0, 0, z} }},
      {{ {-1, 1, 0}, {-1, -1, 0}, {0, 0, z} }},
      // Lower pyramid
      {{ {-1, -1, 0}, {0, 0, -z}, {1, -1, 0} }},
      {{ {1, -1, 0}, {0, 0, -z}, {1, 1, 0} }},
      {{ {1, 1, 0}, {0, 0, -z}, {-1, 1, 0} }},
      {{ {-1, 1, 0}, {0, 0, -z}, {-1, -1, 0} }}
    };
    // clang-format on
  };

  auto polyhedron_vertices_fn = [this]()
  {
    double z = std::max(0.1, (1 + sin(t_ / 2)) / 2);
    // clang-format off
    return std::vector<Eigen::Vector3d>
    {
      {-1, -1, 0},
      {1, -1, 0},
      {1, 1, 0},
      {-1, 1, 0},
      {0, 0, z},
      {0, 0, -z}
    };
    // clang-format on
  };

  auto polyhedron_indices_fn = []()
  {
    // clang-format off
    return std::vector<std::array<size_t, 3>>
    {
      {0, 1, 4},
      {1, 2, 4},
      {2, 3, 4},
      {3, 0, 4},
      {0, 1, 5},
      {1, 2, 5},
      {2, 3, 5},
      {3, 0, 5},
    };
    // clang-format on
  };

  auto polyhedron_colors_fn = [this]()
  {
    double z = std::max(0.1, (1 + cos(t_ / 2)) / 2);
    Eigen::Vector4d color;
    color << mc_rtc::utils::heatmap<Eigen::Vector3d>(0, 1, z), 1;
    auto blue = mc_rtc::gui::Color{0, 0, 1, 1};
    // clang-format off
    return std::vector<std::array<mc_rtc::gui::Color, 3>>
    {
      // Colors for upper pyramid
      {blue, blue, color},
      {blue, blue, color},
      {blue, blue, color},
      {blue, blue, color},
      // Colors for lower pyramid
      {blue, color, blue},
      {blue, color, blue},
      {blue, color, blue},
      {blue, color, blue}
     };
    // clang-format on
  };

  auto polyhedron_vertices_colors_fn = [this]()
  {
    double z = std::max(0.1, (1 + sin(t_ / 2)) / 2);
    Eigen::Vector4d color;
    color << mc_rtc::utils::heatmap<Eigen::Vector3d>(0, 1, z), 1;
    auto blue = mc_rtc::gui::Color{0, 0, 1, 1};
    // clang-format off
    return std::vector<mc_rtc::gui::Color>
    {
      blue,
      blue,
      blue,
      blue,
      color,
      color
    };
    // clang-format on
  };

  mc_rtc::gui::PolyhedronConfig pconfig;
  pconfig.triangle_color = mc_rtc::gui::Color(1, 0, 0, 1.0);
  pconfig.use_triangle_color = false;
  pconfig.show_triangle = true;
  pconfig.show_vertices = true;
  pconfig.show_edges = true;
  pconfig.fixed_edge_color = true;
  pconfig.edge_config.color = mc_rtc::gui::Color::LightGray;
  pconfig.edge_config.width = 0.03;
  static bool publish_as_vertices_triangles = true;
  static bool publish_colors = true;
  auto send_polyhedron = [=]()
  {
    builder.removeElement({"GUI Markers", "Polyhedrons"}, "Polyhedron");
    if(publish_as_vertices_triangles)
    {
      if(publish_colors)
      {
        builder.addElement({"GUI Markers", "Polyhedrons"},
                           mc_rtc::gui::Polyhedron("Polyhedron", pconfig, polyhedron_vertices_fn, polyhedron_indices_fn,
                                                   polyhedron_vertices_colors_fn));
      }
      else
      {
        builder.addElement(
            {"GUI Markers", "Polyhedrons"},
            mc_rtc::gui::Polyhedron("Polyhedron", pconfig, polyhedron_vertices_fn, polyhedron_indices_fn));
      }
    }
    else
    {
      if(publish_colors)
      {
        builder.addElement(
            {"GUI Markers", "Polyhedrons"},
            mc_rtc::gui::Polyhedron("Polyhedron", pconfig, polyhedron_triangles_fn, polyhedron_colors_fn));
      }
      else
      {
        builder.addElement({"GUI Markers", "Polyhedrons"},
                           mc_rtc::gui::Polyhedron("Polyhedron", pconfig, polyhedron_triangles_fn));
      }
    }
  };
  builder.addElement({"GUI Markers", "Polyhedrons"},
                     mc_rtc::gui::Checkbox(
                         "Publish as vertices/indices", []() { return publish_as_vertices_triangles; },
                         [send_polyhedron]
                         {
                           publish_as_vertices_triangles = !publish_as_vertices_triangles;
                           send_polyhedron();
                         }),
                     mc_rtc::gui::Checkbox(
                         "Publish colors", []() { return publish_colors; },
                         [send_polyhedron]
                         {
                           publish_colors = !publish_colors;
                           send_polyhedron();
                         }));
  send_polyhedron();

  builder.addElement(
      {"GUI Markers", "Trajectories"}, mc_rtc::gui::Trajectory("Vector3d", [this]() { return trajectory_; }),
      mc_rtc::gui::Trajectory("PTransformd", {mc_rtc::gui::Color::Green}, [this]() { return poseTrajectory_; }),
      mc_rtc::gui::Trajectory("Live 3D", {mc_rtc::gui::Color::Magenta},
                              [this]() { return Eigen::Vector3d{cos(t_), sin(t_), 1.0}; }),
      mc_rtc::gui::Trajectory("Live transform", {mc_rtc::gui::Color::Black}, [this]()
                              { return lookAt({cos(t_), -1, sin(t_)}, {0, 0, 0}, Eigen::Vector3d::UnitZ()); }));

  mc_rtc::gui::ArrowConfig arrow_config({1., 0., 0.});
  arrow_config.start_point_scale = 0.02;
  arrow_config.end_point_scale = 0.02;
  builder.addElement({"GUI Markers", "Arrows"},
                     mc_rtc::gui::Arrow(
                         "ArrowRO", arrow_config, []() { return Eigen::Vector3d{2, 2, 0}; },
                         []() { return Eigen::Vector3d{2.5, 2.5, 0.5}; }),
                     mc_rtc::gui::Arrow(
                         "Arrow", arrow_config, [this]() { return arrow_start_; },
                         [this](const Eigen::Vector3d & start) { arrow_start_ = start; },
                         [this]() { return arrow_end_; }, [this](const Eigen::Vector3d & end) { arrow_end_ = end; }),
                     mc_rtc::gui::Force(
                         "ForceRO", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(1., 0., 0.)),
                         []() { return sva::ForceVecd(Eigen::Vector3d{0., 0., 0.}, Eigen::Vector3d{10., 0., 100.}); },
                         []() { return sva::PTransformd{Eigen::Vector3d{2, 2, 0}}; }),
                     mc_rtc::gui::Force(
                         "Force", mc_rtc::gui::ForceConfig(mc_rtc::gui::Color(0., 1., 0.)), [this]() { return force_; },
                         [this](const sva::ForceVecd & force) { force_ = force; },
                         []() { return sva::PTransformd{Eigen::Vector3d{2, 2, 0}}; }));
  using Color = mc_rtc::gui::Color;
  using Range = mc_rtc::gui::plot::Range;
  using Style = mc_rtc::gui::plot::Style;
  using Side = mc_rtc::gui::plot::Side;
  auto sin_cos_plot = [this](const std::string & name)
  {
    builder.addPlot(name, mc_rtc::gui::plot::X("t", [this]() { return t_; }),
                    mc_rtc::gui::plot::Y(
                        "sin(t)", [this]() { return std::sin(t_); }, Color::Red),
                    mc_rtc::gui::plot::Y("cos(t)", [this]() { return std::cos(t_); }, Color::Blue));
  };
  add_demo_plot("sin(t)/cos(t)", sin_cos_plot);
  auto discontinuous_plot = [this](const std::string & name)
  {
    builder.addPlot(name, mc_rtc::gui::plot::X("t", [this]() { return t_; }),
                    mc_rtc::gui::plot::Y(
                        "Y",
                        []() -> double
                        {
                          static unsigned int t_i = 0;
                          return t_i++ % 10;
                        },
                        Color::Red));
  };
  add_demo_plot("Discontinuous plot", discontinuous_plot);
  auto demo_style_plot = [this](const std::string & name)
  {
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
  auto fix_axis_plot = [this](const std::string & name)
  {
    builder.addPlot(
        name, mc_rtc::gui::plot::X("t", [this]() { return t_; }), {"Y1", {0, 1}}, // Fix both min and max
        {"Y2", {-Range::inf, 0}}, // Only fix max
        mc_rtc::gui::plot::Y(
            "sin(t)", [this]() { return std::sin(t_); }, Color::Red),
        mc_rtc::gui::plot::Y("cos(t)", [this]() { return std::cos(t_); }, Color::Blue, Style::Solid, Side::Right));
  };
  add_demo_plot("Fix axis", fix_axis_plot);
  using PolygonDescription = mc_rtc::gui::plot::PolygonDescription;
  using Point = std::array<double, 2>;
  using Points = std::vector<Point>;
  auto circle_plot = [this](const std::string & name)
  {
    builder.addXYPlot(name, {"X (m)", {-1, 1}}, {"Y (m)", {-1, 1}},
                      mc_rtc::gui::plot::XY(
                          "Round", [this]() { return std::cos(t_); }, [this]() { return std::sin(t_); }, Color::Red),
                      mc_rtc::gui::plot::Polygon("Square",
                                                 []()
                                                 {
                                                   return PolygonDescription(Points{Point{-1, -1}, Point{-1, 1},
                                                                                    Point{1, 1}, Point{1, -1}},
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
  auto polygons_plot = [this, polygons](const std::string & name)
  { builder.addXYPlot(name, mc_rtc::gui::plot::Polygons("Polygons", [polygons]() { return polygons; })); };
  add_demo_plot("Polygons demo", polygons_plot);
  auto fake_zmp_plot = [this](const std::string & name)
  {
    graph_.reset(t_);
    builder.addXYPlot(name,
                      mc_rtc::gui::plot::XY(
                          "ZMP", [this]() { return graph_.zmp_x; }, [this] { return graph_.zmp_y; },
                          [this]() { return graph_.color(); }),
                      mc_rtc::gui::plot::Polygons("feet", [this]() { return graph_.feet; }));
  };
  add_demo_plot("Fake ZMP", fake_zmp_plot);

  auto dynamic_plot = [this](const std::string & name)
  {
    builder.addPlot(name, mc_rtc::gui::plot::X("t", [this]() { return t_; }));
    n_dynamic_regular_plot = 0;
  };
  auto dynamic_xy_plot = [this](const std::string & name)
  {
    builder.addXYPlot(name);
    n_dynamic_xy_plot = 0;
    // Add an Y-plot, this should return false
    bool added = builder.addPlotData(name, mc_rtc::gui::plot::Y("label", []() { return 0.0; }, Color::Blue));
    assert(!added);
    (void)added;
  };
  auto make_square = [](double x, double y, double size, const Color & color)
  {
    return PolygonDescription(Points{Point{x - size, y - size}, Point{x - size, y + size}, Point{x + size, y + size},
                                     Point{x + size, y - size}},
                              color);
  };
  auto add_dynamic_plot = [&, this](const std::string & name, size_t & n_plots, size_t mod)
  {
    double data = static_cast<double>(n_plots);
    auto label = std::to_string(n_plots);
    if(n_plots % mod == 0)
    {
      double center = static_cast<double>(n_plots);
      builder.addPlotData(name, mc_rtc::gui::plot::XY(
                                    label, [this, center]() { return center + 0.25 * cos(t_); },
                                    [this, center]() { return center + 0.25 * sin(t_); }, Color::Red));
    }
    else if(n_plots % mod == 1)
    {
      auto center = static_cast<double>(n_plots);
      auto poly = make_square(center, center, 0.25, Color::Green).fill(Color::Yellow);
      builder.addPlotData(name, mc_rtc::gui::plot::Polygon(label, [poly]() { return poly; }));
    }
    else if(n_plots % mod == 2)
    {
      auto center = static_cast<double>(n_plots);
      std::vector<PolygonDescription> polys;
      polys.push_back(make_square(center - 0.5, center - 0.5, 0.125, Color::Black).fill(Color::Cyan));
      polys.push_back(make_square(center - 0.5, center + 0.5, 0.125, Color::Black).fill(Color::Cyan));
      polys.push_back(make_square(center + 0.5, center + 0.5, 0.125, Color::Black).fill(Color::Cyan));
      polys.push_back(make_square(center + 0.5, center - 0.5, 0.125, Color::Black).fill(Color::Cyan));
      builder.addPlotData(name, mc_rtc::gui::plot::Polygons(label, [polys]() { return polys; }));
    }
    else
    {
      builder.addPlotData(name, mc_rtc::gui::plot::Y(label, [data]() { return data; }, Color::Blue));
    }
    n_plots += 1;
  };
  add_demo_plot("Dynamic Regular Plot", dynamic_plot, "Add data", add_dynamic_plot, n_dynamic_regular_plot, 4);
  add_demo_plot("Dynamic XY Plot", dynamic_xy_plot, "Add XY data", add_dynamic_plot, n_dynamic_xy_plot, 3);

  auto toggle_chunky_xy_plot = [this]()
  {
    if(chunky_xy_data_.has_plot)
    {
      chunky_xy_data_.has_plot = false;
      builder.removePlot("Chunky xy plot");
      return;
    }
    chunky_xy_data_.has_plot = true;
    builder.addXYPlot("Chunky xy plot", mc_rtc::gui::plot::XYChunk(
                                            "Data",
                                            [this](std::vector<std::array<double, 2>> & points)
                                            {
                                              if(!chunky_xy_data_.add_data) { return; }
                                              double t0 = chunky_xy_data_.t;
                                              double tF = 0.1;
                                              for(double t = t0; t < t0 + 1.0; t += 0.01)
                                              {
                                                points.push_back({t, cos(t)});
                                              }
                                              chunky_xy_data_.add_data = false;
                                              chunky_xy_data_.t += 1.0;
                                            },
                                            mc_rtc::gui::Color::Green));
  };
  builder.addElement({}, mc_rtc::gui::ElementsStacking::Horizontal,
                     mc_rtc::gui::Button("Add chunky XY plot", [toggle_chunky_xy_plot]() { toggle_chunky_xy_plot(); }),
                     mc_rtc::gui::Button("Send chunk", [this]() { chunky_xy_data_.add_data = true; }));

  switch_visual("sphere");

  configureRobotLoader();
  robots_ = mc_rbdyn::loadRobot(*mc_rbdyn::RobotLoader::get_robot_module("JVRC1"));
  auto & robot = robots_->robot("jvrc1");
  robot.posW({Eigen::Vector3d{-2.0, 0.0, robot.posW().translation().z()}});
  builder.addElement(
      {"Robot"}, mc_rtc::gui::Robot("jvrc1", [this]() -> const mc_rbdyn::Robot & { return robots_->robot("jvrc1"); }));
}

void SampleServer::switch_visual(const std::string & choice)
{
  visualChoice_ = choice;
  builder.removeCategory({"Visual"});
  builder.addElement({"Visual"}, mc_rtc::gui::ComboInput(
                                     "Choice", {"sphere", "box", "cylinder", "mesh"}, [this]() -> const std::string &
                                     { return visualChoice_; }, [this](const std::string & c) { switch_visual(c); }));
  if(choice == "sphere")
  {
    visual_ = mc_rtc::makeVisualSphere(sphereRadius_, {1, 0, 0, 0.2});
    builder.addElement({"Visual"}, mc_rtc::gui::NumberInput(
                                       "radius", [this]() { return sphereRadius_; },
                                       [this](double r)
                                       {
                                         auto & s = mc_rtc::getVisualSphere(visual_);
                                         s.radius = r;
                                         sphereRadius_ = r;
                                       }));
  }
  else if(choice == "box")
  {
    visual_ = mc_rtc::makeVisualBox(boxDim_, {0, 0, 1, 0.2});
    builder.addElement({"Visual"},
                       mc_rtc::gui::ArrayInput(
                           "dimensions", {"x", "y", "z"}, [this]() -> const Eigen::Vector3d & { return boxDim_; },
                           [this](const Eigen::Vector3d & v)
                           {
                             auto & b = mc_rtc::getVisualBox(visual_);
                             b.size = v;
                             boxDim_ = v;
                           }));
  }
  else if(choice == "cylinder")
  {
    visual_ = mc_rtc::makeVisualCylinder(cylinderRadius_, cylinderLength_, {0, 1, 0, 0.2});
    builder.addElement({"Visual"},
                       mc_rtc::gui::NumberInput(
                           "radius", [this]() { return cylinderRadius_; },
                           [this](double r)
                           {
                             auto & c = mc_rtc::getVisualCylinder(visual_);
                             c.radius = r;
                             cylinderRadius_ = r;
                           }),
                       mc_rtc::gui::NumberInput(
                           "length", [this]() { return cylinderLength_; },
                           [this](double r)
                           {
                             auto & c = mc_rtc::getVisualCylinder(visual_);
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
      if(robot.module()._visual.count(b.name())) { choices.push_back(b.name()); }
    }
    builder.addElement({"Visual"}, mc_rtc::gui::ComboInput(
                                       "body", choices, [this]() { return robotVisual_; },
                                       [this](const std::string & s)
                                       {
                                         robotVisual_ = s;
                                         switch_visual("mesh");
                                       }));
  }
  builder.addElement({"Visual", "Position"},
                     mc_rtc::gui::Transform(
                         "position", [this]() -> const sva::PTransformd & { return visualPos_; },
                         [this](const sva::PTransformd & p) { visualPos_ = p; }));
  if(choice == "mesh") { return; }
  builder.addElement({"Visual"}, mc_rtc::gui::Visual(
                                     choice, [this]() -> const rbd::parsers::Visual & { return visual_; },
                                     [this]() -> const sva::PTransformd & { return visualPos_; }));
  builder.addElement({"Visual", "Ellipsoid"},
                     mc_rtc::gui::Ellipsoid(
                         "Fixed size/Fixed color", Eigen::Vector3d(0.25, 0.5, 1.0),
                         []() { return sva::PTransformd(Eigen::Vector3d(-4, 0, 1)); }, mc_rtc::gui::Color::Blue),
                     mc_rtc::gui::Ellipsoid(
                         "Fixed size/Varying color", Eigen::Vector3d(0.25, 0.5, 1.0),
                         []() { return sva::PTransformd(Eigen::Vector3d(-4, 1, 1)); },
                         [this]()
                         {
                           auto color = mc_rtc::gui::Color::Yellow;
                           color.a = (1 + cos(t_)) / 2;
                           return color;
                         }),
                     mc_rtc::gui::Ellipsoid(
                         "Varying size/Fixed color", [this]() -> Eigen::Vector3d
                         { return Eigen::Vector3d(0.25, 0.5, 1.0) * (1 + (1 + cos(t_)) / 2); },
                         []() { return sva::PTransformd(Eigen::Vector3d(-4, 2, 1)); }),
                     mc_rtc::gui::Ellipsoid(
                         "Varying size/Varying color", [this]() -> Eigen::Vector3d
                         { return Eigen::Vector3d(0.25, 0.5, 1.0) * (1 + (1 + sin(t_)) / 2); },
                         []() { return sva::PTransformd(Eigen::Vector3d(-4, 3, 1)); },
                         [this]()
                         {
                           auto color = mc_rtc::gui::Color::Green;
                           color.a = (1 + sin(t_)) / 2;
                           return color;
                         }));
  builder.addElement({"Visual", "Cylinder"},
                     mc_rtc::gui::Cylinder(
                         "Fixed size/Fixed color", {0.125, 1.0},
                         []() { return sva::PTransformd(Eigen::Vector3d(-3, 0, 1)); }, mc_rtc::gui::Color::Blue),
                     mc_rtc::gui::Cylinder(
                         "Fixed size/Varying color", {0.125, 1.0},
                         []() { return sva::PTransformd(Eigen::Vector3d(-3, 1, 1)); },
                         [this]()
                         {
                           auto color = mc_rtc::gui::Color::Yellow;
                           color.a = (1 + cos(t_)) / 2;
                           return color;
                         }),
                     mc_rtc::gui::Cylinder(
                         "Varying size/Fixed color",
                         [this]() -> mc_rtc::gui::CylinderParameters
                         {
                           double m = (1 + (1 + cos(t_)) / 2);
                           return {0.125 * m, 0.5 * m};
                         },
                         []() { return sva::PTransformd(Eigen::Vector3d(-3, 2, 1)); }),
                     mc_rtc::gui::Cylinder(
                         "Varying size/Varying color",
                         [this]() -> mc_rtc::gui::CylinderParameters
                         {
                           double m = (1 + (1 + sin(t_)) / 2);
                           return {0.125 * m, 0.5 * m};
                         },
                         []() { return sva::PTransformd(Eigen::Vector3d(-3, 3, 1)); },
                         [this]()
                         {
                           auto color = mc_rtc::gui::Color::Green;
                           color.a = (1 + sin(t_)) / 2;
                           return color;
                         }));
  builder.addElement({"Visual", "Box"},
                     mc_rtc::gui::Box(
                         "Fixed size/Fixed color", Eigen::Vector3d(0.25, 0.5, 1.0),
                         []() { return sva::PTransformd(Eigen::Vector3d(-2, 0, 1)); }, mc_rtc::gui::Color::Blue),
                     mc_rtc::gui::Box(
                         "Fixed size/Varying color", Eigen::Vector3d(0.25, 0.5, 1.0),
                         []() { return sva::PTransformd(Eigen::Vector3d(-2, 1, 1)); },
                         [this]()
                         {
                           auto color = mc_rtc::gui::Color::Yellow;
                           color.a = (1 + cos(t_)) / 2;
                           return color;
                         }),
                     mc_rtc::gui::Box(
                         "Varying size/Fixed color", [this]() -> Eigen::Vector3d
                         { return Eigen::Vector3d(0.25, 0.5, 1.0) * (1 + (1 + cos(t_)) / 2); },
                         []() { return sva::PTransformd(Eigen::Vector3d(-2, 2, 1)); }),
                     mc_rtc::gui::Box(
                         "Varying size/Varying color", [this]() -> Eigen::Vector3d
                         { return Eigen::Vector3d(0.25, 0.5, 1.0) * (1 + (1 + sin(t_)) / 2); },
                         []() { return sva::PTransformd(Eigen::Vector3d(-2, 3, 1)); },
                         [this]()
                         {
                           auto color = mc_rtc::gui::Color::Green;
                           color.a = (1 + sin(t_)) / 2;
                           return color;
                         }));
  builder.addElement({"Visual", "Sphere"},
                     mc_rtc::gui::Sphere(
                         "Fixed radius/Fixed color", 0.25, []() { return sva::PTransformd(Eigen::Vector3d(-1, 0, 1)); },
                         mc_rtc::gui::Color::Blue),
                     mc_rtc::gui::Sphere(
                         "Fixed radius/Varying color", 0.25,
                         []() { return sva::PTransformd(Eigen::Vector3d(-1, 1, 1)); },
                         [this]()
                         {
                           auto color = mc_rtc::gui::Color::Yellow;
                           color.a = (1 + cos(t_)) / 2;
                           return color;
                         }),
                     mc_rtc::gui::Sphere(
                         "Varying radius/Fixed color", [this]() { return 0.25 * (1 + (1 + cos(t_)) / 2); },
                         []() { return sva::PTransformd(Eigen::Vector3d(-1, 2, 1)); }),
                     mc_rtc::gui::Sphere(
                         "Varying radius/Varying color", [this]() { return 0.25 * (1 + (1 + sin(t_)) / 2); },
                         []() { return sva::PTransformd(Eigen::Vector3d(-1, 3, 1)); },
                         [this]()
                         {
                           auto color = mc_rtc::gui::Color::Green;
                           color.a = (1 + sin(t_)) / 2;
                           return color;
                         }));
}

template<typename T>
void SampleServer::add_demo_plot(const std::string & name, T callback)
{
  bool has_plot = false;
  builder.addElement({}, mc_rtc::gui::Button("Add " + name + " plot",
                                             [has_plot, callback, name, this]() mutable
                                             {
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

template<typename T, typename U>
void SampleServer::add_demo_plot(const std::string & name,
                                 T callback,
                                 const std::string & dynamic_label,
                                 U dynamic_callback,
                                 size_t & n_plots,
                                 size_t mod)
{
  bool has_plot = false;
  builder.addElement({}, mc_rtc::gui::ElementsStacking::Horizontal,
                     mc_rtc::gui::Button("Add " + name + " plot",
                                         [has_plot, callback, name, this]() mutable
                                         {
                                           if(has_plot) { builder.removePlot(name); }
                                           else
                                           {
                                             callback(name);
                                           }
                                           has_plot = !has_plot;
                                         }),
                     mc_rtc::gui::Button(dynamic_label, [dynamic_callback, name, mod, &n_plots]()
                                         { dynamic_callback(name, n_plots, mod); }));
}

void SampleServer::publish()
{
  graph_.update(t_);
  TestServer::publish();
}

void SampleServer::make_table(size_t s)
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
    for(size_t j = 1; j <= s; ++j) { vec.push_back(std::pow(d, j)); }
    d += 1.1;
  }
  table_header = std::move(header);
  table_format = std::move(format);
  table_data = std::move(data);
}

int main()
{
  SampleServer server;
  server.loop();
  return 0;
}
