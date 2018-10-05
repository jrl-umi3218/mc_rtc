#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui_api.h>

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
  double r = 0.0;
  double g = 0.0;
  double b = 0.0;
  double a = 1.0;

  void load(const mc_rtc::Configuration & config)
  {
    r = config("r");
    g = config("g");
    b = config("b");
    a = config("a");
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
    config("arrow_shaft_diam", arrow_shaft_diam);
    config("arrow_head_diam", arrow_head_diam);
    config("arrow_head_len", arrow_head_len);
    config("arrow_scale", arrow_scale);
  }

  operator mc_rtc::Configuration() const
  {
    mc_rtc::Configuration config;
    config.add("color", color);
    config.add("arrow_shaft_diam", arrow_shaft_diam);
    config.add("arrow_head_diam", arrow_head_diam);
    config.add("arrow_head_len", arrow_head_len);
    config.add("arrow_scale", arrow_scale);
    return config;
  }

  double arrow_head_diam = 0.015;
  double arrow_head_len = 0.05;
  double arrow_shaft_diam = 0.015;
  double arrow_scale = 0.0015;
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

} // namespace gui
} // namespace mc_rtc
