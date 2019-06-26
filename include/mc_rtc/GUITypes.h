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

  operator mc_rtc::Configuration() const
  {
    mc_rtc::Configuration config;
    config.add("r", r);
    config.add("g", g);
    config.add("b", b);
    config.add("a", a);
    return config;
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

  void save(mc_rtc::Configuration & out) const
  {
    out.add("color", color);
    out.add("width", width);
    switch(style)
    {
      case LineStyle::Dotted:
        out.add("style", "dotted");
        break;
      default:
        out.add("style", "solid");
        break;
    }
  }

  operator mc_rtc::Configuration() const
  {
    mc_rtc::Configuration out;
    save(out);
    return out;
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

  operator mc_rtc::Configuration() const
  {
    mc_rtc::Configuration config;
    config.add("color", color);
    config.add("shaft_diam", shaft_diam);
    config.add("head_diam", head_diam);
    config.add("head_len", head_len);
    config.add("scale", scale);
    config.add("start_point_scale", start_point_scale);
    config.add("end_point_scale", end_point_scale);
    return config;
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

  operator mc_rtc::Configuration() const
  {
    auto config = ArrowConfig::operator mc_rtc::Configuration();
    config.add("force_scale", force_scale);
    return config;
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

  operator mc_rtc::Configuration() const
  {
    mc_rtc::Configuration config;
    config.add("scale", scale);
    config.add("color", color);
    return config;
  }

  Color color;
  double scale = 0.02;
};

} // namespace gui
} // namespace mc_rtc
