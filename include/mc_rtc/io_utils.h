/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <algorithm>
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
 */
template<typename Container>
std::string to_string(const Container & c, const std::string & delimeter = ", ")
{
  std::string s{""};
  std::for_each(c.cbegin(), c.cend(), [&](const std::string & e) {
    if(!s.empty())
    {
      s += delimeter;
    }
    s += e;
  });
  return s;
}

/** Converts a container to a string
 *
 * Example:
 *\code{.cpp}
 * const auto availabeRobots = mc_rtc::io::to_string(robots(), [](const mc_rbdyn::Robot & r) -> const std::string & {
 *return r.name(); }); \endcode
 *
 * @tparam Container An iterable container whose unerlying type is convertible
 * to std::string. The container must define Container::value_type.
 * @tparam GetT Functor that converts an element of type Container::value_type to std::string
 *
 * @tparam Container A container whose underlying type is convertible to
 * @param c Container to convert
 *
 * @return A string of all the container elements.
 */
template<typename Container, typename GetT>
std::string to_string(const Container & c, GetT get_value, const std::string & delimiter = ", ")
{
  std::string s{""};
  std::for_each(c.cbegin(), c.cend(), [&](const typename Container::value_type & e) {
    if(!s.empty())
    {
      s += delimiter;
    }
    s += get_value(e);
  });
  return s;
}

} // namespace io
} // namespace mc_rtc
