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
  Color(const mc_rtc::Configuration & config)
  {
    load(config);
  }
  double r = 1.0;
  double g = 0.0;
  double b = 0.0;
  double a = 1.0;

  bool operator==(const Color & rhs) const
  {
    return r == rhs.r && g == rhs.g && b == rhs.b && a == rhs.a;
  }

  bool operator!=(const Color & rhs) const
  {
    return !(*this == rhs);
  }

  void load(const mc_rtc::Configuration & config)
  {
    std::array<double, 4> data = config;
    r = data[0];
    g = data[1];
    b = data[2];
    a = data[3];
  }

  static constexpr size_t write_size()
  {
    return 1;
  }

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
};

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
  LineConfig(const mc_rtc::Configuration & config)
  {
    load(config);
  }

  void load(const mc_rtc::Configuration & config)
  {
    color.load(config[0]);
    width = config[1];
    style = LineStyle(static_cast<typename std::underlying_type<LineStyle>::type>(config[2]));
  }

  static constexpr size_t write_size()
  {
    return 1;
  }

  void write(mc_rtc::MessagePackBuilder & out) const
  {
    out.start_array(3);
    color.write(out);
    out.write(width);
    out.write(static_cast<typename std::underlying_type<LineStyle>::type>(style));
    out.finish_array();
  }
};

struct MC_RTC_GUI_DLLAPI ArrowConfig
{
  ArrowConfig() : color(0., 1., 0.) {}
  ArrowConfig(const Color & color) : color(color) {}
  ArrowConfig(const mc_rtc::Configuration & config)
  {
    load(config);
  };

  void load(const mc_rtc::Configuration & config)
  {
    head_diam = config[0];
    head_len = config[1];
    shaft_diam = config[2];
    scale = config[3];
    start_point_scale = config[4];
    end_point_scale = config[5];
    color.load(config[6]);
  }

  static constexpr size_t write_size()
  {
    return 1;
  }

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

struct ForceConfig : public ArrowConfig
{
  ForceConfig() : ArrowConfig() {}
  ForceConfig(const Color & color) : ArrowConfig(color) {}
  ForceConfig(const mc_rtc::Configuration & config)
  {
    load(config);
  };

  void load(const mc_rtc::Configuration & config)
  {
    ArrowConfig::load(config[0]);
    force_scale = config[1];
  }

  static constexpr size_t write_size()
  {
    return 1;
  }

  void write(mc_rtc::MessagePackBuilder & out) const
  {
    out.start_array(2);
    ArrowConfig::write(out);
    out.write(force_scale);
    out.finish_array();
  }

  double force_scale = 0.0015;
};

struct PointConfig
{
  PointConfig() {}
  PointConfig(const Color & color, double scale = 0.02) : color(color), scale(scale) {}
  PointConfig(const mc_rtc::Configuration & config)
  {
    load(config);
  }

  void load(const mc_rtc::Configuration & config)
  {
    color.load(config[0]);
    scale = config[1];
  }

  static constexpr size_t write_size()
  {
    return 1;
  }

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
} // namespace mc_rtc
