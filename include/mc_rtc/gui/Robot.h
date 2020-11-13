/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/gui/types.h>

#include <mc_rbdyn/Robot.h>

namespace mc_rtc
{

namespace gui
{

namespace details
{

/** Robot should display a robot model in the environment
 *
 * The element provides the following data to the client:
 * - the parameters passed to RobotLoader to get the RobotModule (vector<string>)
 * - the current robot configuration (vector<vector<double>>)
 *
 * \tparam GetT Should return an mc_rbdyn::Robot
 */
template<typename GetT>
struct RobotImpl : public Element
{
  static constexpr auto type = Elements::Robot;

  RobotImpl(const std::string & name, GetT get_fn) : Element(name), get_fn_(get_fn)
  {
    static_assert(CheckReturnType<GetT, mc_rbdyn::Robot>::value, "Robot element must return an mc_rbdyn::Robot");
  }

  static constexpr size_t write_size()
  {
    return Element::write_size() + 3;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    const mc_rbdyn::Robot & robot = get_fn_();
    Element::write(builder);
    builder.write(robot.module().parameters());
    builder.write(robot.mbc().q);
    builder.write(robot.posW());
  }

private:
  GetT get_fn_;
};

} // namespace details

/** Helper function to create a RobotImpl */
template<typename GetT>
details::RobotImpl<GetT> Robot(const std::string & name, GetT get_fn)
{
  return details::RobotImpl<GetT>(name, get_fn);
}

} // namespace gui

} // namespace mc_rtc
