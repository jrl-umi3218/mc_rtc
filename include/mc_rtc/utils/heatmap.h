#pragma once
#include <Eigen/Core>

namespace mc_rtc
{
namespace utils
{

/**
 * @brief Compute rgb heatmap colors expressed between 0 and 1
 *
 * Minimum value is represented as pure blue
 * Maximum value is represented as pure red
 * In-between values vary between blue and red
 *
 * @param minimum Minimum value that the heatmap should represent
 * @param maximum Maximum value that the heatmap should represent
 * @param value Current value in [minimum, maximum] range. Values outside of
 * this range will be displayed as either blue or red.
 *
 * @return [r,g,b] values
 */
template<typename T>
T heatmap(double minimum, double maximum, double value)
{
  if(value > maximum)
  {
    return {1, 0, 0};
  }
  if(value < minimum)
  {
    return {0, 0, 1};
  }

  const auto ratio = 2 * (value - minimum) / (maximum - minimum);
  const auto b = std::max(0., (1 - ratio));
  const auto r = std::max(0., (ratio - 1));
  const auto g = 1 - b - r;
  return {r, g, b};
}

} // namespace utils
} // namespace mc_rtc
