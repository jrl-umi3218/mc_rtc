#include <mc_rbdyn/RobotFrame.h>

#include <mc_rbdyn/Robot.h>

namespace mc_rbdyn
{

RobotFrame::RobotFrame(NewRobotFrameToken tkn, const std::string & name, Robot & robot, const std::string & body)
: mc_rtc::shared<RobotFrame, Frame>(tkn, name), robot_(robot)
{
  if(!robot.hasBody(body))
  {
    mc_rtc::log::error_and_throw(
        "Attempted to create a frame for {} attached to body {} which is not part of this robot", robot.name(), body);
  }
  bodyMbcIdx_ = robot.bodyIndexByName(body);
  sensor_ = robot.findBodyForceSensor(body);
}

RobotFrame::RobotFrame(NewRobotFrameToken tkn,
                       const std::string & name,
                       RobotFrame & parent,
                       sva::PTransformd X_p_f,
                       bool baked)
: mc_rtc::shared<RobotFrame, Frame>(tkn, name, parent, X_p_f, baked), robot_(parent.robot()),
  bodyMbcIdx_(parent.bodyMbcIdx_), sensor_(parent.sensor_)
{
}

const std::string & RobotFrame::body() const noexcept
{
  return robot_.mb().body(static_cast<int>(bodyMbcIdx_)).name();
}

sva::PTransformd RobotFrame::position() const noexcept
{
  if(!parent_)
  {
    return position_ * robot_.mbc().bodyPosW[bodyMbcIdx_];
  }
  return position_ * static_cast<RobotFrame *>(parent_.get())->position();
}

sva::MotionVecd RobotFrame::velocity() const noexcept
{
  if(!parent_)
  {
    return position_ * robot_.mbc().bodyVelW[bodyMbcIdx_];
  }
  return position_ * static_cast<RobotFrame *>(parent_.get())->velocity();
}

const ForceSensor & RobotFrame::forceSensor() const
{
  if(!sensor_)
  {
    mc_rtc::log::error_and_throw("No force sensor attached to {} in {}", name_, robot_.name());
  }
  return *sensor_;
}

sva::ForceVecd RobotFrame::wrench() const
{
  if(!sensor_)
  {
    mc_rtc::log::error_and_throw("No force sensor attached to {} in {}", name_, robot_.name());
  }
  if(!parent_)
  {
    // Find the transformation from the sensor to the frame
    auto X_fsactual_body = [this]() {
      if(sensor_->parent() == body())
      {
        return position_ * sensor_->X_fsactual_parent();
      }
      else
      {
        const auto & X_0_body = this->position();
        const auto & X_0_parent = robot_.frame(sensor_->parentBody()).position();
        const auto X_parent_body = X_0_body * X_0_parent.inv();
        return position_ * X_parent_body * sensor_->X_fsactual_parent();
      }
    }();
    return X_fsactual_body.dualMul(sensor_->wrenchWithoutGravity(robot_));
  }
  else
  {
    return position_.dualMul(static_cast<RobotFrame *>(parent_.get())->wrench());
  }
}

Eigen::Vector2d RobotFrame::cop(double min_pressure) const
{
  // We don't check if the force sensor is attached, this will be done when we get the wrench and the stack will tell
  // the rest of the story
  const sva::ForceVecd w_surf = wrench();
  const double pressure = w_surf.force()(2);
  if(pressure < min_pressure)
  {
    return Eigen::Vector2d::Zero();
  }
  const Eigen::Vector3d & tau_surf = w_surf.couple();
  return Eigen::Vector2d(-tau_surf(1) / pressure, +tau_surf(0) / pressure);
}

Eigen::Vector3d RobotFrame::copW(double min_pressure) const
{
  Eigen::Vector3d cop_s;
  cop_s << cop(min_pressure), 0.;
  const sva::PTransformd X_0_s = position();
  return X_0_s.translation() + X_0_s.rotation().transpose() * cop_s;
}

RobotFramePtr RobotFrame::makeFrame(const std::string & name, const sva::PTransformd & X_p_f, bool baked)
{
  return robot_.makeFrame(name, *this, X_p_f, baked);
}

sva::PTransformd RobotFrame::X_b_f() const noexcept
{
  if(parent_)
  {
    return position_ * static_cast<RobotFrame *>(parent_.get())->X_b_f();
  }
  return position_;
}

} // namespace mc_rbdyn
