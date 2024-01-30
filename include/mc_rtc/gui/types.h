/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/** This file contains classes that are used to configure the visual styles of
 * various elements in the GUI */

#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui/api.h>
#include <mc_rtc/logging.h>

namespace mc_rtc
{

namespace gui
{
struct MC_RTC_GUI_DLLAPI Color
{
  Color() {}
  Color(double r, double g, double b, double a = 1.0) : r(r), g(g), b(b), a(a) {}
  Color(const Eigen::Vector3d & color) : r(color.x()), g(color.y()), b(color.z()) {}
  Color(const Eigen::Vector4d & color) : r(color.x()), g(color.y()), b(color.z()), a(color[3]) {}
  Color(const mc_rtc::Configuration & config) { fromConfig(config); }
  double r = 1.0;
  double g = 0.0;
  double b = 0.0;
  double a = 1.0;

  bool operator==(const Color & rhs) const { return r == rhs.r && g == rhs.g && b == rhs.b && a == rhs.a; }

  bool operator!=(const Color & rhs) const { return !(*this == rhs); }

  void fromMessagePack(const mc_rtc::Configuration & config)
  {
    std::array<double, 4> data = config;
    r = data[0];
    g = data[1];
    b = data[2];
    a = data[3];
  }

  void fromConfig(const mc_rtc::Configuration & config)
  {
    try
    {
      std::array<double, 4> color = config;
      r = color[0];
      g = color[1];
      b = color[2];
      a = color[3];
    }
    catch(mc_rtc::Configuration::Exception & e)
    {
      e.silence();
      try
      {
        std::string color = config;
        if(ColorMap.count(color)) { *this = ColorMap.at(color); }
        else
        {
          auto msg = fmt::format("No color named {} ", color);
          mc_rtc::log::error(msg);
          throw mc_rtc::Configuration::Exception(msg, config);
        }
      }
      catch(mc_rtc::Configuration::Exception & e)
      {
        e.silence();
        auto msg = std::string{"Color is neither an array of size 4, nor a valid color string"};
        mc_rtc::log::error(msg);
        throw mc_rtc::Configuration::Exception(msg, config);
      }
    }
  }

  mc_rtc::Configuration saveConfig() const
  {
    auto conf = mc_rtc::Configuration::rootArray();
    conf.push(r);
    conf.push(g);
    conf.push(b);
    conf.push(a);
    return conf;
  }

  static constexpr size_t write_size() { return 1; }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.start_array(4);
    builder.write(r);
    builder.write(g);
    builder.write(b);
    builder.write(a);
    builder.finish_array();
  }

  static const Color White;
  static const Color Black;
  static const Color Red;
  static const Color Green;
  static const Color Blue;
  static const Color Cyan;
  static const Color Magenta;
  static const Color Yellow;
  static const Color Gray;
  static const Color LightGray;
  static const std::map<std::string, Color> ColorMap;
};
} // namespace gui

template<>
struct ConfigurationLoader<mc_rtc::gui::Color>
{
  static mc_rtc::gui::Color load(const mc_rtc::Configuration & config)
  {
    mc_rtc::gui::Color color;
    color.fromConfig(config);
    return color;
  }

  static mc_rtc::Configuration save(const mc_rtc::gui::Color & color) { return color.saveConfig(); }
};

namespace gui
{
enum class LineStyle
{
  Solid,
  Dotted
};

struct MC_RTC_GUI_DLLAPI LineConfig
{
  Color color = {1, 0, 0};
  double width = 0.01;
  LineStyle style = LineStyle::Solid;

  LineConfig() {}
  LineConfig(const Color & color, double width = 0.01, const LineStyle & style = LineStyle::Solid)
  : color(color), width(width), style(style)
  {
  }
  LineConfig(const mc_rtc::Configuration & config) { fromConfig(config); }

  void fromMessagePack(const mc_rtc::Configuration & config)
  {
    color.fromMessagePack(config[0]);
    width = config[1];
    style = LineStyle(static_cast<typename std::underlying_type<LineStyle>::type>(config[2]));
  }

  void fromConfig(const mc_rtc::Configuration & config)
  {
    config("color", color);
    config("width", width);
    if(config.has("style"))
    {
      auto s = config("style");
      if(s == "solid") { style = LineStyle::Solid; }
      else if(s == "dotted") { style = LineStyle::Dotted; }
    }
  }

  mc_rtc::Configuration saveConfig() const
  {
    mc_rtc::Configuration config;
    config.add("color", color);
    config.add("width", width);
    if(style == LineStyle::Solid) { config.add("style", "solid"); }
    else if(style == LineStyle::Dotted) { config.add("style", "dotted"); }
    return config;
  }

  static constexpr size_t write_size() { return 1; }

  void write(mc_rtc::MessagePackBuilder & out) const
  {
    out.start_array(3);
    color.write(out);
    out.write(width);
    out.write(static_cast<typename std::underlying_type<LineStyle>::type>(style));
    out.finish_array();
  }
};
} // namespace gui

template<>
struct ConfigurationLoader<mc_rtc::gui::LineConfig>
{
  static mc_rtc::gui::LineConfig load(const mc_rtc::Configuration & config)
  {
    mc_rtc::gui::LineConfig line;
    line.fromConfig(config);
    return line;
  }

  static mc_rtc::Configuration save(const mc_rtc::gui::LineConfig & line) { return line.saveConfig(); }
};

namespace gui
{
struct MC_RTC_GUI_DLLAPI ArrowConfig
{
  ArrowConfig() : color(0., 1., 0.) {}
  ArrowConfig(const Color & color) : color(color) {}
  ArrowConfig(const mc_rtc::Configuration & config) { fromConfig(config); };

  void fromMessagePack(const mc_rtc::Configuration & config)
  {
    head_diam = config[0];
    head_len = config[1];
    shaft_diam = config[2];
    scale = config[3];
    start_point_scale = config[4];
    end_point_scale = config[5];
    color.fromMessagePack(config[6]);
  }

  void fromConfig(const mc_rtc::Configuration & config)
  {
    config("head_diam", head_diam);
    config("head_len", head_len);
    config("shaft_diam", shaft_diam);
    config("scale", scale);
    config("start_point_scale", start_point_scale);
    config("end_point_scale", end_point_scale);
    config("color", color);
  }

  mc_rtc::Configuration saveConfig() const
  {
    mc_rtc::Configuration conf;
    conf.add("head_diam", head_diam);
    conf.add("head_len", head_len);
    conf.add("shaft_diam", shaft_diam);
    conf.add("scale", scale);
    conf.add("start_point_scale", start_point_scale);
    conf.add("end_point_scale", end_point_scale);
    conf.add("color", color);
    return conf;
  }

  static constexpr size_t write_size() { return 1; }

  void write(mc_rtc::MessagePackBuilder & out) const
  {
    out.start_array(7);
    out.write(head_diam);
    out.write(head_len);
    out.write(shaft_diam);
    out.write(scale);
    out.write(start_point_scale);
    out.write(end_point_scale);
    color.write(out);
    out.finish_array();
  }

  double head_diam = 0.015;
  double head_len = 0.05;
  double shaft_diam = 0.015;
  double scale = 0.0015;
  double start_point_scale = 0.0;
  double end_point_scale = 0.0;
  Color color;
};
} // namespace gui

template<>
struct ConfigurationLoader<mc_rtc::gui::ArrowConfig>
{
  static mc_rtc::gui::ArrowConfig load(const mc_rtc::Configuration & config)
  {
    mc_rtc::gui::ArrowConfig arrow;
    arrow.fromConfig(config);
    return arrow;
  }

  static mc_rtc::Configuration save(const mc_rtc::gui::ArrowConfig & arrow) { return arrow.saveConfig(); }
};

namespace gui
{
struct ForceConfig : public ArrowConfig
{
  ForceConfig() : ArrowConfig() {}
  ForceConfig(const Color & color) : ArrowConfig(color) {}
  ForceConfig(const mc_rtc::Configuration & config) { fromConfig(config); };

  void fromMessagePack(const mc_rtc::Configuration & config)
  {
    ArrowConfig::fromMessagePack(config[0]);
    force_scale = config[1];
  }

  void fromConfig(const mc_rtc::Configuration & config)
  {
    ArrowConfig::fromConfig(config);
    config("force_scale", force_scale);
  }

  mc_rtc::Configuration saveConfig() const
  {
    mc_rtc::Configuration conf = ArrowConfig::saveConfig();
    conf.add("force_scale", force_scale);
    return conf;
  }

  static constexpr size_t write_size() { return 1; }

  void write(mc_rtc::MessagePackBuilder & out) const
  {
    out.start_array(2);
    ArrowConfig::write(out);
    out.write(force_scale);
    out.finish_array();
  }

  double force_scale = 0.0015;
};

} // namespace gui

template<>
struct ConfigurationLoader<mc_rtc::gui::ForceConfig>
{
  static mc_rtc::gui::ForceConfig load(const mc_rtc::Configuration & config)
  {
    mc_rtc::gui::ForceConfig force;
    force.fromConfig(config);
    return force;
  }

  static mc_rtc::Configuration save(const mc_rtc::gui::ForceConfig & force) { return force.saveConfig(); }
};

namespace gui
{
struct PointConfig
{
  PointConfig() {}
  PointConfig(const Color & color, double scale = 0.02) : color(color), scale(scale) {}
  PointConfig(const mc_rtc::Configuration & config) { fromConfig(config); }

  void fromMessagePack(const mc_rtc::Configuration & config)
  {
    color.fromMessagePack(config[0]);
    scale = config[1];
  }

  void fromConfig(const mc_rtc::Configuration & config)
  {
    color.fromConfig(config);
    config("scale", scale);
  }

  mc_rtc::Configuration saveConfig() const
  {
    mc_rtc::Configuration conf;
    conf.add("color", color);
    conf.add("scale", scale);
    return conf;
  }

  static constexpr size_t write_size() { return 1; }

  void write(mc_rtc::MessagePackBuilder & out) const
  {
    out.start_array(2);
    color.write(out);
    out.write(scale);
    out.finish_array();
  }

  Color color;
  double scale = 0.02;
};
} // namespace gui

template<>
struct ConfigurationLoader<mc_rtc::gui::PointConfig>
{
  static mc_rtc::gui::PointConfig load(const mc_rtc::Configuration & config)
  {
    mc_rtc::gui::PointConfig point;
    point.fromConfig(config);
    return point;
  }

  static mc_rtc::Configuration save(const mc_rtc::gui::PointConfig & point) { return point.saveConfig(); }
};

namespace gui
{

struct MC_RTC_GUI_DLLAPI PolyhedronConfig
{
  PolyhedronConfig() {}

  PolyhedronConfig(const mc_rtc::Configuration & config) { fromConfig(config); }

  void fromConfig(const mc_rtc::Configuration & config)
  {
    config("triangle_color", triangle_color);
    config("show_triangle", show_triangle);
    config("use_triangle_color", use_triangle_color);
    config("edges", edge_config);
    config("show_edges", show_edges);
    config("fixed_edge_color", fixed_edge_color);
    config("vertices", vertices_config);
    config("show_vertices", show_vertices);
    config("fixed_vertices_color", fixed_vertices_color);
  }

  mc_rtc::Configuration saveConfig() const
  {
    mc_rtc::Configuration conf;
    conf.add("triangle_color", triangle_color);
    conf.add("show_triangle", show_triangle);
    conf.add("use_triangle_color", use_triangle_color);
    conf.add("edges", edge_config);
    conf.add("show_edges", show_edges);
    conf.add("fixed_edge_color", fixed_edge_color);
    conf.add("vertices", vertices_config);
    conf.add("show_vertices", show_vertices);
    conf.add("fixed_vertices_color", fixed_vertices_color);
    return conf;
  }

  void fromMessagePack(const mc_rtc::Configuration & config)
  {
    triangle_color.fromMessagePack(config[0]);
    show_triangle = config[1];
    use_triangle_color = config[2];
    edge_config.fromMessagePack(config[3]);
    show_edges = config[4];
    fixed_edge_color = config[5];
    vertices_config.fromMessagePack(config[6]);
    show_vertices = config[7];
    fixed_vertices_color = config[8];
  }

  static constexpr size_t write_size() { return 1; }

  void write(mc_rtc::MessagePackBuilder & out) const
  {
    out.start_array(9);
    triangle_color.write(out);
    out.write(show_triangle);
    out.write(use_triangle_color);

    edge_config.write(out);
    out.write(show_edges);
    out.write(fixed_edge_color);

    vertices_config.write(out);
    out.write(show_vertices);
    out.write(fixed_vertices_color);
    out.finish_array();
  }

  ///< Default triangle face color
  Color triangle_color;
  bool show_triangle = true;
  ///< When true, use triangle face color from triangle_color. Else interpolate between each vertex color
  bool use_triangle_color = false;
  LineConfig edge_config;
  bool show_edges = true;
  ///< When true, use edge colors from edges_config. Else interpolate between each vertex color
  bool fixed_edge_color = true;
  PointConfig vertices_config;
  bool show_vertices = true;
  ///< When true, use vertices colors from vertices_config. Else use the vertex color
  ///< passed along with the triangles
  bool fixed_vertices_color = true;
};

/** Provides full information about a robot kinematic and dynamic state */
struct MC_RTC_GUI_DLLAPI RobotMsgData
{
  RobotMsgData() = default;
  RobotMsgData(const std::vector<std::string> & params,
               const Eigen::VectorXd & q,
               const Eigen::VectorXd & alpha,
               const Eigen::VectorXd & alphaD,
               const Eigen::VectorXd & tau,
               const std::vector<sva::ForceVecd> & forces,
               const sva::PTransformd & posW)
  : parameters(params), q(q), alpha(alpha), alphaD(alphaD), tau(tau), forces(forces), posW(posW)
  {
  }

  std::vector<std::string> parameters;
  Eigen::VectorXd q;
  Eigen::VectorXd alpha;
  Eigen::VectorXd alphaD;
  Eigen::VectorXd tau;
  std::vector<sva::ForceVecd> forces;
  sva::PTransformd posW;
};

} // namespace gui

template<>
struct ConfigurationLoader<mc_rtc::gui::PolyhedronConfig>
{
  static mc_rtc::gui::PolyhedronConfig load(const mc_rtc::Configuration & config)
  {
    mc_rtc::gui::PolyhedronConfig polyhedron;
    polyhedron.fromConfig(config);
    return polyhedron;
  }

  static mc_rtc::Configuration save(const mc_rtc::gui::PolyhedronConfig & polyhedron)
  {
    return polyhedron.saveConfig();
  }
};

} // namespace mc_rtc
