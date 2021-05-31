/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <algorithm>
#include <functional>
#include <limits>
#include <sstream>
#include <string>

namespace mc_rtc
{
namespace io
{

/** Converts a container to a string
 *
 * @tparam Container A container whose underlying type is convertible to
 * std::string. The container must define Container::value_type.
 * @param c Container to convert
 * @param delimiter Separator between the elements. By default they are
 * coma-separated
 *
 * @return A string of all the container elements.
 *
 * @anchor to_string
 */
template<typename Container,
         typename std::enable_if<!std::is_arithmetic<typename Container::value_type>::value, int>::type = 0>
std::string to_string(const Container & c, const std::string & delimiter = ", ")
{
  if(c.cbegin() == c.cend())
  {
    return "";
  }
  std::string out = *c.cbegin();
  for(auto it = std::next(c.cbegin()); it != c.cend(); ++it)
  {
    out += delimiter;
    out += *it;
  }
  return out;
}

/**
 * @brief Variant of \ref to_string that converts numeric types to string
 *
 * @param precision Number of digits to keep. The default precision corresponds
 * to the number of decimal digits that can be represented without change.
 */
template<typename Container,
         typename std::enable_if<std::is_arithmetic<typename Container::value_type>::value, int>::type = 0>
std::string to_string(const Container & c,
                      const std::string & delimiter = ", ",
                      const unsigned precision = std::numeric_limits<typename Container::value_type>::digits10)
{
  if(c.cbegin() == c.cend())
  {
    return "";
  }
  std::ostringstream out;
  out.precision(precision);
  out << std::fixed << *c.cbegin();
  for(auto it = std::next(c.cbegin()); it != c.cend(); ++it)
  {
    out << delimiter;
    out << std::fixed << *it;
  }
  return out.str();
}

/** Converts a container to a string
 *
 * Example:
 *\code{.cpp}
 * const auto availabeRobots = mc_rtc::io::to_string(robots(), [](const mc_rbdyn::Robot & r) -> const std::string & {
 *return r.name(); });
 \endcode
 *
 * @tparam Container An iterable container whose unerlying type is convertible
 * to std::string. The container must define Container::value_type.
 *
 * @param c Container to convert
 * @param get_value Lambda or functor that converts a single element from the container (of type Container::value_type)
 to std::string
 *
 * @return A string of all the container elements.
 */
template<typename Container,
         typename Callback,
         typename std::enable_if<!std::is_convertible<Callback, std::string>::value, int>::type = 0>
std::string to_string(const Container & c, Callback && get_value, const std::string & delimiter = ", ")
{
  if(c.cbegin() == c.cend())
  {
    return "";
  }
  std::string out = get_value(*c.cbegin());
  for(auto it = std::next(c.cbegin()); it != c.cend(); ++it)
  {
    out += delimiter;
    out += get_value(*it);
  }
  return out;
}

} // namespace io
} // namespace mc_rtc
