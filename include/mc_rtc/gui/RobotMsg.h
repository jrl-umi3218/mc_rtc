/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/gui/types.h>

#include <mc_rbdyn/Robot.h>

namespace mc_rtc::gui
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
struct RobotMsgImpl : public Element
{
  static constexpr auto type = Elements::RobotMsg;

  RobotMsgImpl(const std::string & name, GetT get_fn) : Element(name), get_fn_(get_fn)
  {
    static_assert(CheckReturnType<GetT, mc_rbdyn::Robot>::value, "Robot element must return an mc_rbdyn::Robot");
  }

  static constexpr size_t write_size() { return Element::write_size() + 7; }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    const mc_rbdyn::Robot & robot = get_fn_();
    update(robot);
    Element::write(builder);
    builder.write(robot.module().parameters());
    builder.write(msg_.q);
    builder.write(msg_.alpha);
    builder.write(msg_.alphaD);
    builder.write(msg_.tau);
    builder.write(robot.mbc().force);
    builder.write(robot.posW());
  }

private:
  GetT get_fn_;
  RobotMsgData msg_;

  void update(const mc_rbdyn::Robot & robot)
  {
    msg_.q.resize(robot.mb().nrParams());
    rbd::paramToVector(robot.mbc().q, msg_.q);
    msg_.alpha.resize(robot.mb().nrDof());
    rbd::paramToVector(robot.mbc().alpha, msg_.alpha);
    msg_.alphaD.resize(msg_.alpha.size());
    rbd::paramToVector(robot.mbc().alphaD, msg_.alphaD);
    msg_.tau.resize(msg_.alpha.size());
    rbd::paramToVector(robot.mbc().jointTorque, msg_.tau);
  }
};

} // namespace details

/** Helper function to create a RobotImpl */
template<typename GetT>
auto RobotMsg(const std::string & name, GetT get_fn)
{
  return details::RobotMsgImpl(name, get_fn);
}

} // namespace mc_rtc::gui
