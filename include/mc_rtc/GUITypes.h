/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui_api.h>
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

  void load(const mc_rtc::Configuration & config)
  {
    config("r", r);
    config("g", g);
    config("b", b);
    config("a", a);
  }

  static constexpr size_t write_size()
  {
    return 4;
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.write(r);
    builder.write(g);
    builder.write(b);
    builder.write(a);
  }
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
    config("color", color);
    config("width", width);
    std::string styleStr = config("style", std::string("solid"));
    if(styleStr == "solid")
    {
      style = LineStyle::Solid;
    }
    else if(styleStr == "dotted")
    {
      style = LineStyle::Dotted;
    }
    else
    {
      LOG_WARNING("Unknown line style (" << styleStr << "), defaulting to solid")
      style = LineStyle::Solid;
    }
  }

  static constexpr size_t write_size()
  {
    return Color::write_size() + 2;
  }

  void write(mc_rtc::MessagePackBuilder & out) const
  {
    color.write(out);
    out.write(width);
    out.write(static_cast<typename std::underlying_type<LineStyle>::type>(style));
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
    config("color", color);
    config("shaft_diam", shaft_diam);
    config("head_diam", head_diam);
    config("head_len", head_len);
    config("scale", scale);
    config("start_point_scale", start_point_scale);
    config("end_point_scale", end_point_scale);
  }

  static constexpr size_t write_size()
  {
    return Color::write_size() + 7;
  }

  void write(mc_rtc::MessagePackBuilder & out) const
  {
    color.write(out);
    out.write(shaft_diam);
    out.write(shaft_diam);
    out.write(head_diam);
    out.write(head_len);
    out.write(scale);
    out.write(start_point_scale);
    out.write(end_point_scale);
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
    ArrowConfig::load(config);
    config("force_scale", force_scale);
  }

  static constexpr size_t write_size()
  {
    return ArrowConfig::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & out) const
  {
    ArrowConfig::write(out);
    out.write(force_scale);
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
    config("color", color);
    config("scale", scale);
  }

  static constexpr size_t write_size()
  {
    return Color::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & out) const
  {
    color.write(out);
    out.write(scale);
  }

  Color color;
  double scale = 0.02;
};

} // namespace gui
} // namespace mc_rtc
