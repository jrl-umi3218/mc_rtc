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
  /** This plot type is expected to return a list of polygons */
  Polygons,
  /** This plot type return (X, Y) couples */
  AbscissaOrdinate
};

template<typename T>
constexpr bool is_Abscissa()
{
  return T::type == Type::Abscissa;
}

template<typename T>
constexpr bool is_not_Abscissa()
{
  return T::type != Type::Abscissa;
}

template<typename T, typename T2, typename... Args>
constexpr bool is_not_Abscissa()
{
  return is_not_Abscissa<T>() && is_not_Abscissa<T2, Args...>();
}

template<typename T>
constexpr bool is_2d()
{
  return T::type != Type::Abscissa && T::type != Type::Ordinate;
}

template<typename T, typename T2, typename... Args>
constexpr bool is_2d()
{
  return is_2d<T>() && is_2d<T2, Args...>();
}

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

  void fromMessagePack(const mc_rtc::Configuration & config)
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

  AxisConfiguration & min(double min)
  {
    range.min = min;
    return *this;
  }

  AxisConfiguration & max(double max)
  {
    range.max = max;
    return *this;
  }

  void fromMessagePack(const mc_rtc::Configuration & config)
  {
    name = static_cast<std::string>(config[0]);
    range.fromMessagePack(config[1]);
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
  /** Point display: display a single value */
  Point
};

/** Which side an ordinate should be attached to (default: Left) */
enum class MC_RTC_GUI_DLLAPI Side
{
  Left,
  Right
};

/** Describe a polygon to be plotted */
struct MC_RTC_GUI_DLLAPI PolygonDescription
{
  PolygonDescription() : PolygonDescription({}, {}) {}

  PolygonDescription(const std::vector<std::array<double, 2>> & points, Color outline)
  : points_(points), outline_(outline)
  {
  }

  bool operator==(const PolygonDescription & rhs) const
  {
    return points_ == rhs.points_ && outline_ == rhs.outline_ && style_ == rhs.style_ && fill_ == rhs.fill_
           && closed_ == rhs.closed_;
  };

  bool operator!=(const PolygonDescription & rhs) const
  {
    return !(*this == rhs);
  }

  // clang-format off
  std::vector<std::array<double, 2>> & points() { return points_; }
  Color & outline() { return outline_; };
  Style & style() { return style_; };
  Color & fill() { return fill_; };
  bool closed() { return closed_; };

  const std::vector<std::array<double, 2>> & points() const { return points_; }
  const Color & outline() const { return outline_; };
  const Style & style() const { return style_; };
  const Color & fill() const { return fill_; };
  bool closed() const { return closed_; };

  PolygonDescription & outline(const Color & outline) { outline_ = outline; return *this; };
  PolygonDescription & style(const Style & style) { style_ = style; return *this; };
  PolygonDescription & fill(const Color & fill) { fill_ = fill; return *this; };
  PolygonDescription & closed(bool closed) { closed_ = closed; return *this; };
  // clang-format on

  void fromMessagePack(const mc_rtc::Configuration & data)
  {
    points_ = data[0];
    outline_.fromMessagePack(data[1]);
    style_ = static_cast<Style>(static_cast<uint64_t>(data[2]));
    fill_.fromMessagePack(data[3]);
    closed_ = data[4];
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.start_array(5);
    builder.write(points_);
    outline_.write(builder);
    builder.write(static_cast<uint64_t>(style_));
    fill_.write(builder);
    builder.write(closed_);
    builder.finish_array();
  }

private:
  /** List of points in the polygon */
  std::vector<std::array<double, 2>> points_;
  /** Outline color */
  Color outline_;
  /** Outline style */
  Style style_ = Style::Solid;
  /** Fill-color, setting alpha to 0 should disable filling */
  Color fill_ = Color(0, 0, 0, 0);
  /** If true, close the polygon */
  bool closed_ = true;
};

} // namespace plot

} // namespace gui

} // namespace mc_rtc
