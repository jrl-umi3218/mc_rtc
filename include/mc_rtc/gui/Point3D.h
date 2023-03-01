/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/gui/types.h>

namespace mc_rtc::gui
{

namespace details
{

/** Point3D should display a 3D point in the environment
 *
 * A PointConfig is provided to control how the point is displayed
 *
 * The point can be edited if \tparam SetT is not std::nullptr_t
 *
 * It will also trigger an ArrayLabel or ArrayInput with {"x", "y", "z"} labels
 *
 * \tparam GetT Should return an Eigen::Vector3d
 *
 * \tparam SetT Will be called when the point is moved or the ArrayInput is triggered if non std::nullptr_t
 */
template<typename GetT, typename SetT = std::nullptr_t>
struct Point3DImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::Point3D;

  Point3DImpl(const std::string & name, const PointConfig & config, GetT get_fn, SetT set_fn = nullptr)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn), config_(config)
  {
    static_assert(details::CheckReturnType<GetT, Eigen::Vector3d>::value,
                  "Point3D element position callback must return an Eigen::Vector3d");
  }

  /** Invalid element */
  Point3DImpl() {}

  static constexpr size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 1 + PointConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CommonInputImpl<GetT, SetT>::write(builder);
    // True for read-only
    builder.write(std::is_same_v<SetT, std::nullptr_t>);
    config_.write(builder);
  }

private:
  PointConfig config_;
};

} // namespace details

/** Helper function to create a read-only Point3DImpl */
template<typename GetT, std::enable_if_t<std::is_invocable_v<GetT>, int> = 0>
auto Point3D(const std::string & name, GetT get_fn)
{
  return details::Point3DImpl(name, {}, get_fn);
}

/** Helper function to create a Point3DImpl */
template<typename GetT, typename SetT>
auto Point3D(const std::string & name, GetT get_fn, SetT set_fn)
{
  return details::Point3DImpl(name, {}, get_fn, set_fn);
}

/** Helper function to create a read-only Point3DImpl with configuration */
template<typename GetT, std::enable_if_t<std::is_invocable_v<GetT>, int> = 0>
auto Point3D(const std::string & name, const PointConfig & config, GetT get_fn)
{
  return details::Point3DImpl(name, config, get_fn);
}

/** Helper function to create a Point3DImpl with configuration */
template<typename GetT, typename SetT>
auto Point3D(const std::string & name, const PointConfig & config, GetT get_fn, SetT set_fn)
{
  return details::Point3DImpl(name, config, get_fn, set_fn);
}

/** Helper function to build a Point3D from a variable */
template<typename T>
auto Point3DRO(const std::string & name, T && value)
{
  return Point3D(name, details::read(std::forward<T>(value)));
}

/** Helper function to build a Point3D from a variable */
template<typename T>
auto Point3DRO(const std::string & name, const PointConfig & config, T && value)
{
  return Point3D(name, config, details::read(std::forward<T>(value)));
}

/** Helper function to build a Point3D from a variable */
template<typename T, std::enable_if_t<!std::is_invocable_v<T>, int> = 0>
auto Point3D(const std::string & name, T & value)
{
  return Point3D(name, details::read(value), details::write(value));
}

/** Helper function to build a Point3D from a variable */
template<typename T, std::enable_if_t<!std::is_invocable_v<T>, int> = 0>
auto Point3D(const std::string & name, const PointConfig & config, T & value)
{
  return Point3D(name, config, details::read(value), details::write(value));
}

} // namespace mc_rtc::gui
