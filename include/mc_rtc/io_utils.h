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
 * std::string
 * @param c Container to convert
 *
 * @return A coma-separated string of all the container elements.
 */
template<typename Container>
std::string to_string(const Container & c)
{
  std::string s{""};
  std::for_each(c.cbegin(), c.cend(), [&](const std::string & e) {
    if(!s.empty())
    {
      s += ", ";
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
 * @tparam Container An iterable container of arbitrary type
 * The container must define Container::value_type
 * @tparam GetT Functor that converts an element of type Container::value_type to std::string
 *
 * @tparam Container A container whose underlying type is convertible to
 * @param c Container to convert
 *
 * @return A coma-separated string of all the container elements.
 */
template<typename Container, typename GetT>
std::string to_string(const Container & c, GetT get_value)
{
  std::string s{""};
  std::for_each(c.cbegin(), c.cend(), [&](const typename Container::value_type & e) {
    if(!s.empty())
    {
      s += ", ";
    }
    s += get_value(e);
  });
  return s;
}

} // namespace io
} // namespace mc_rtc
