/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/types.h>

#include <limits>

namespace mc_rtc
{

namespace gui
{

namespace plot
{

/** Describe plot types for the client */
enum class Plot
{
  /** Identify a plot that provides an abscissa */
  Standard = 0,
  /** Identify a plot that provides XY legends */
  XY
};

/** Distinguish plot types */
enum class Type
{
  /** This plot type is expected to return X values only */
  Abscissa,
  /** This plot type is expected to return Y values only */
  Ordinate,
  /** This plot type is expected to return a list of points to plot */
  Polygon,
  /** This plot type return (X, Y) couples */
  AbscissaOrdinate
};

/** Range for abscissa or ordinate display
 *
 * Setting the min to -inf or max to +inf tells the client to compute the
 * related limit (default)
 */
struct MC_RTC_GUI_DLLAPI Range
{
  static constexpr double inf = std::numeric_limits<double>::infinity();
  double min = -inf;
  double max = inf;

  Range() = default;
  Range(double min, double max) : min(min), max(max) {}

  void load(const mc_rtc::Configuration & config)
  {
    std::array<double, 2> data = config;
    min = data[0];
    max = data[1];
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.start_array(2);
    builder.write(min);
    builder.write(max);
    builder.finish_array();
  }
};

/** Describe the configuration of an axis */
struct MC_RTC_GUI_DLLAPI AxisConfiguration
{
  std::string name;
  Range range;

  AxisConfiguration() = default;
  AxisConfiguration(const std::string & name) : AxisConfiguration(name, {}) {}
  AxisConfiguration(Range range) : AxisConfiguration("", range) {}
  AxisConfiguration(const std::string & name, Range range) : name(name), range(range) {}

  void load(const mc_rtc::Configuration & config)
  {
    name = static_cast<std::string>(config[0]);
    range.load(config[1]);
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.start_array(2);
    builder.write(name);
    range.write(builder);
    builder.finish_array();
  }
};

/** How to display the plot */
enum class MC_RTC_GUI_DLLAPI Style
{
  /** Solid lines */
  Solid,
  /** Dotted lines */
  Dotted,
  /** Dashed lines */
  Dashed,
  /** Point display */
  Scatter
};

/** Which side an ordinate should be attached to (default: Left) */
enum class MC_RTC_GUI_DLLAPI Side
{
  Left,
  Right
};

} // namespace plot

} // namespace gui

} // namespace mc_rtc
