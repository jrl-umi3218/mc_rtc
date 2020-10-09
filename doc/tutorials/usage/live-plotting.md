---
layout: tutorials
toc: true
---

In some instances, you may want to display graphs while a controller is running (for example, when tuning the gains of a stabilizer). One option to do this is to open the on-going log and plot the data you're interested in. With long running experiments this can become cumbersome as the log size grows as does the time needed to open the log and process it.

The better option, presented in this tutorial, is to let your controller display an on-going log.

<h5 class="no_toc">Note</h5>

For the sake of clarity, in the examples, we will assume the following code snippet is present:

{% capture source %}
// Brings in every type we need
#include <mc_rtc/gui/plot.h>

// Make shorter names for types we will use a lot
using Color = mc_rtc::gui::Color;
using PolygonDescription = mc_rtc::gui::plot::PolygonDescription;
using Range = mc_rtc::gui::plot::Range;
using Style = mc_rtc::gui::plot::Style;
using Side = mc_rtc::gui::plot::Side;
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

## Two simple examples to get started

The two examples show how to create a graph with a shared abscissa and a graph without a shared abscissa.

In the first case, we must provide an absicssa constructed by `mc_rtc::gui::plot::X` and plot data constructed by `mc_rtc::gui::plot::Y`, `mc_rtc::gui::plot::XY` or `mc_rtc::gui::plot::Polygon(s)`. For the remainder of this document we will call this type of plots "regular plots".

{% capture source %}
gui()->addPlot(
  "sin(t)",
  mc_rtc::gui::plot::X("t", [this]() { return t; }),
  mc_rtc::gui::plot::Y("sin(t)", [this]() { return sin(t); }, Color::Red)
);
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

In the second case, we only need to proivde data constructed by `mc_rtc::gui::plot::XY` or `mc_rtc::gui::plot::Polygon(s)`. For the remainder of this document we will call this type of pots "XY plots".

{% capture source %}
{% raw %}
gui()->addXYPlot(
  "Circle in a square",
  mc_rtc::gui::plot::XY("Circle",
                        [this]() { return cos(t); }, [this]() { return sin(t); },
                        Color::Red),
  mc_rtc::gui::plot::Polygon("Square",
                             []() { return PolygonDescription({{-1, 1}, {-1, 1}, {1, 1}, {1, -1}}, Color::Blue); })
);
{% endraw %}
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

Both types of plots can be removed in the same way:

{% capture source %}
gui()->removePlot("sin(t)");
gui()->removePlot("Circle in a square");
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

That's the gist of it! In the next sections we will take a closer look at each of the elements introduced in the examples.

## `mc_rtc::gui::plot::AxisConfiguration`

This class lets you manipulate the name and the range of a given axis. By default the range is `[-inf, +inf]` which tells the GUI client to automatically guess the limits.

### Configuring the abscissa (X axis)

#### In a regular plot

In a regular plot, the abscissa configuration is provided by `mc_rtc::gui::plot::X`. In our first example, we only provided the name as the first argument but it can also accept a full configuration. For example:

{% capture source %}
double t0 = t;
gui()->addPlot(
  "sin(t)",
  // Minimum value on the axis will be t0
  // Maximum value on the axis will be t0 + 10
  mc_rtc::gui::plot::X({"t", {t, t + 10}}, [this]() { return t; }),
  mc_rtc::gui::plot::Y("sin(t)", [this]() { return sin(t); }, Color::Red)
);
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

#### In a XY plot

In a XY plot, the abscissa configuration is optional, if you wish to provide it then an `mc_rtc::gui::plot::AxisConfiguration` must be provided as the first argument of `addXYPlot`. For example:

{% capture source %}
gui()->addXYPlot(
  "Circle",
  // Shows an alternative way to set the limits
  AxisConfiguration("X").min(-1).max(1),
  mc_rtc::gui::plot::XY("Circle",
                        [this]() { return cos(t); }, [this]() { return sin(t); },
                        Color::Red)
);
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

### Configuring the ordinates (Left Y axis and right Y axis)

The ordinates can be configured by providing an `AxisConfiguration` object for the left and right axis:

#### Left Y axis

- in a regular plot, the left Y axis configuration must be provided after the abscissa.
- in a XY plot, the left Y axis configuration must be provided after the X axis configuration

#### Right Y axis

In both cases, the right Y axis configuration must be provided after the left Y axis configuration.

#### Example

{% capture source %}
// Providing only the left Y axis configuration
double t0 = t;
gui()->addPlot(
  "sin(t)",
  mc_rtc::gui::plot::X({"t", {t, t + 10}}, [this]() { return t; }),
  AxisConfiguration("Y", {-1, 1}),
  mc_rtc::gui::plot::Y("sin(t)", [this]() { return sin(t); }, Color::Red)
);

// Providing both
gui()->addXYPlot(
  "Circle",
  // Shows an alternative way to set the limits
  AxisConfiguration("X").min(-1).max(1),
  AxisConfiguration("Left Y"),
  AxisConfiguration("Right Y").max(10),
  mc_rtc::gui::plot::XY("Circle",
                        [this]() { return cos(t); }, [this]() { return sin(t); },
                        Color::Red)
);
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

## `mc_rtc::gui::plot::X`

There is not much to add at this point, you must provide two elements:
1. the name or the `AxisConfiguration` for the abscissa
2. a callback that returns a `double`

## `mc_rtc::gui::plot::Y`

This has three required arguments and two optional arguments:

1. a name that will be used for the plot legend (**required**)
2. a callback that returns a `double` (**required**)
3. a `Color` or a callback that returns a `Color` (**required**)
4. a line `Style`, this defaults to `Style::Solid`
5. a `Side` to put the data on left Y axis or right Y axis, this defaults to `Side::Left`

Convenience functions are provided to specify the `Style` and `Side` after constructing the object.

#### About `mc_rtc::gui::plot::Style`

Four line styles are available:

1. `Style::Solid` is a solid line;
2. `Style::Dashed` is a dashed line;
3. `Style::Dotted` is a dotted line;
4. `Style::Point` replaces the line with a single point;

#### Example

{% capture source %}
gui()->addPlot(
  "cos(t)/sin(t)",
  mc_rtc::gui::plot::X("t", [this]() { return t; }),
  mc_rtc::gui::plot::Y("cos(t)", [this]() { return cos(t); }, Color::Red, Style::Dotted),
  // Specify the side without changing the default style
  mc_rtc::gui::plot::Y("sin(t)", [this]() { return sin(t); }, Color::Blue).side(Side::Right)
);
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

## `mc_rtc::gui::plot::XY`

This has four required arguments and two optional arguments:

1. a name that will be used for the plot legend (**required**)
2. a callback that returns a `double` for the abscissa value (x) (**required**)
3. a callback that returns a `double` for the ordinate value (y) (**required**)
4. a `Color` or a callback that returns a `Color` (**required**)
5. a line `Style`, this defaults to `Style::Solid`
6. a `Side` to put the data on left Y axis or right Y axis, this defaults to `Side::Left`

Convenience functions are provided to specify the `Style` and `Side` after constructing the object.

## `mc_rtc::gui::plot::Polygon`

This has two required arguments and one optional argument:

1. a name that will be used for the plot legend (**required**)
2. a callback that returns a `PolygonDescription`, we introduce this next (**required**)
3. a `Side` to put the data on left Y axis or right Y axis, this defaults to `Side::Left`

A convenience function is provided to specify the `Side` after constructing the object.

### `mc_rtc::gui::plot::PolygonDescription`

This object holds the following information:
- `points()` is a list of points (i.e. an `std::vector<std::array<double, 2>>`)
- `outline()` is the `Color` of the polygon outline
- `style()` is the line `Style` of the polygon outline (defaults to `Style::Solid`)
- `fill()` is the `Color` used to fill the polygon when it is closed (defaults to transparent, i.e. no filling)
- `closed()` tells whether the polygon is closed (defaults to true)

_Note: if the polygon is closed, you don't need to repeat the first point at the end of `points()`, this will be handled by the client._

#### Example

{% capture source %}
{% raw %}
auto redSquareBlueFill =
  PolygonDescription({{-1, -1}, {-1, 1}, {1, 1}, {1, -1}}, Color::Red).fill(Color::Blue);
auto purpleTriangleYellowFill =
  PolygonDescription({{1, 0}, {1.5, 2}, {2, -2}}, Color::Magenta).fill(Color::Yellow);
auto cyanRectangle =
  PolygonDescription({{-2, -2}, {2, -2}, {2, -3}, {-2, -3}}, Color::Cyan);
{% endraw %}
{% endcapture %}
{% include show_source.html lang="cpp" source=source %}

_Note: since `mc_rtc::gui::plot::Polygon` callback returns a `PolygonDescription` you are free to change the display style of this polygon anytime while the plot is active._

## `mc_rtc::gui::plot::Polygons`

This is similar to `mc_rtc::gui::plot::Polygon` but the callback must return an `std::vector<PolygonDescription>`. This allows you to provide a changing set of polygons while the plot is active to create simple animations.
