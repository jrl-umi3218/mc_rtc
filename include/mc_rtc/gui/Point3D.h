/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/gui/types.h>

namespace mc_rtc
{

namespace gui
{

/** Point3D should display a 3D point in the environment
 *
 * A PointConfig is provided to control how the point should be displayed
 *
 * With this variant, the point cannot be edited
 *
 * It will also trigger an ArrayLabel with {"x", "y", "z"} labels
 *
 * \tparam GetT Should return an Eigen::Vector3d
 */
template<typename GetT>
struct Point3DROImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Point3D;

  Point3DROImpl(const std::string & name, const PointConfig & config, GetT get_fn)
  : DataElement<GetT>(name, get_fn), config_(config)
  {
    static_assert(details::CheckReturnType<GetT, Eigen::Vector3d>::value,
                  "Point3D element position callback must return an Eigen::Vector3d");
  }

  static constexpr size_t write_size()
  {
    return DataElement<GetT>::write_size() + 1 + PointConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    DataElement<GetT>::write(builder);
    builder.write(true); // Read-only
    config_.write(builder);
  }

  /** Invalid element */
  Point3DROImpl() {}

private:
  PointConfig config_;
};

/** Point3D should display a 3D point in the environment
 *
 * A PointConfig is provided to control how the point is displayed
 *
 * With this variant, the point can be edited
 *
 * It will also trigger an ArrayInput with {"x", "y", "z"} labels
 *
 * \tparam GetT Should return an Eigen::Vector3d
 *
 * \tparam SetT Will be called when the point is moved or the ArrayInput is triggered
 */
template<typename GetT, typename SetT>
struct Point3DImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::Point3D;

  Point3DImpl(const std::string & name, const PointConfig & config, GetT get_fn, SetT set_fn)
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
    builder.write(false); // Not read-only
    config_.write(builder);
  }

private:
  PointConfig config_;
};

/** Helper function to create a Point3DROImpl */
template<typename GetT>
Point3DROImpl<GetT> Point3D(const std::string & name, GetT get_fn)
{
  return Point3DROImpl<GetT>(name, {}, get_fn);
}

/** Helper function to create a Point3DImpl */
template<typename GetT, typename SetT>
Point3DImpl<GetT, SetT> Point3D(const std::string & name, GetT get_fn, SetT set_fn)
{
  return Point3DImpl<GetT, SetT>(name, {}, get_fn, set_fn);
}

/** Helper function to create a Point3DROImpl with configuration */
template<typename GetT>
Point3DROImpl<GetT> Point3D(const std::string & name, const PointConfig & config, GetT get_fn)
{
  return Point3DROImpl<GetT>(name, config, get_fn);
}

/** Helper function to create a Point3DImpl with configuration */
template<typename GetT, typename SetT>
Point3DImpl<GetT, SetT> Point3D(const std::string & name, const PointConfig & config, GetT get_fn, SetT set_fn)
{
  return Point3DImpl<GetT, SetT>(name, config, get_fn, set_fn);
}

} // namespace gui

} // namespace mc_rtc
