/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/SplineTrajectoryTask.h>

#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Rotation.h>
#include <mc_rtc/gui/Transform.h>

namespace mc_tasks
{

template<typename Derived>
SplineTrajectoryTask<Derived>::SplineTrajectoryTask(const mc_rbdyn::RobotFrame & frame,
                                                    double duration,
                                                    double stiffness,
                                                    double weight,
                                                    const Eigen::Matrix3d & target,
                                                    const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
: TrajectoryTaskGeneric<tasks::qp::TransformTask>(frame, stiffness, weight), frame_(frame), duration_(duration),
  oriSpline_(duration, frame.position().rotation(), target, oriWp), dimWeightInterpolator_(), stiffnessInterpolator_(),
  dampingInterpolator_()
{
  type_ = "trajectory";
  name_ = "trajectory_" + frame.robot().name() + "_" + frame.name();

  finalize(robots.mbs(), static_cast<int>(rIndex), frame.body(), frame.position(), frame.X_b_f());
}

template<typename Derived>
std::function<bool(const mc_tasks::MetaTask &, std::string &)> SplineTrajectoryTask<Derived>::buildCompletionCriteria(
    double dt,
    const mc_rtc::Configuration & config) const
{

  if(config.has("timeElapsed"))
  {
    bool useDuration = config("timeElapsed");
    if(useDuration)
    {
      return [](const mc_tasks::MetaTask & t, std::string & out) {
        const auto & self = static_cast<const SplineTrajectoryBase &>(t);
        if(self.timeElapsed())
        {
          out += "duration";
          return true;
        }
        return false;
      };
    }
  }
  if(config.has("wrench"))
  {
    sva::ForceVecd target_w = config("wrench");
    Eigen::Vector6d target = target_w.vector();
    Eigen::Vector6d dof = Eigen::Vector6d::Ones();
    for(int i = 0; i < 6; ++i)
    {
      if(std::isnan(target(i)))
      {
        dof(i) = 0.;
        target(i) = 0.;
      }
      else if(target(i) < 0)
      {
        dof(i) = -1.;
      }
    }
    return [dof, target](const mc_tasks::MetaTask & t, std::string & out) {
      const auto & self = static_cast<const SplineTrajectoryTask &>(t);
      Eigen::Vector6d w = self.frame_->wrench().vector();
      for(int i = 0; i < 6; ++i)
      {
        if(dof(i) * fabs(w(i)) < target(i))
        {
          return false;
        }
      }
      out += "wrench";
      return true;
    };
  }
  return TrajectoryBase::buildCompletionCriteria(dt, config);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  TrajectoryBase::load(solver, config);
  if(config.has("gainsInterpolation"))
  {
    /**
     * Load gains from config as vector<pair<double,Vector6d>> or vector<pair<double, double>>
     **/
    auto genValues = [this](const mc_rtc::Configuration & c, const std::string & key,
                            const Eigen::Vector6d & startGains) {
      const auto & conf = c(key);
      std::vector<std::pair<double, mc_rtc::Configuration>> values = conf;
      std::vector<std::pair<double, Eigen::Vector6d>> out;
      out.reserve(values.size());
      for(const auto & v : values)
      {
        try
        {
          if(v.second.size())
          {
            Eigen::Vector6d vec = v.second;
            out.push_back(std::make_pair(v.first, vec));
          }
          else
          {
            out.push_back({v.first, Eigen::Vector6d::Constant(static_cast<double>(v.second))});
          }
        }
        catch(mc_rtc::Configuration::Exception & e)
        {
          mc_rtc::log::critical("[{}] {} interpolation values are expected to be either vector<pair<double, "
                                "Vector6d>> or vector<pair<double, double>>",
                                name(), key);
          throw e;
        }
      }
      // Add initial gain if missing from configuration
      if(out.empty())
      {
        out.push_back({0., startGains});
      }
      else if(!out.empty() && out.front().first > 0.)
      {
        out.insert(out.begin(), {0, startGains});
      }
      return out;
    };

    auto gconfig = config("gainsInterpolation");
    if(gconfig.has("stiffness"))
    {
      stiffnessInterpolation(genValues(gconfig, "stiffness", this->dimStiffness()));
    }

    if(gconfig.has("damping"))
    {
      dampingInterpolation(genValues(gconfig, "damping", this->dimDamping()));
    }

    if(gconfig.has("dimWeight"))
    {
      dimWeightInterpolation(genValues(gconfig, "dimWeight", this->dimWeight()));
    }
  }
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::update(mc_solver::QPSolver & solver)
{
  auto & spline = static_cast<Derived &>(*this).spline();
  spline.samplingPoints(samples_);
  spline.update();

  if(!paused_)
  {
    // Interpolate dimWeight/stiffness/damping
    interpolateGains();

    // Interpolate position
    auto res = spline.splev(currTime_, 2);
    Eigen::Vector3d & pos = res[0];
    Eigen::Vector3d & vel = res[1];
    Eigen::Vector3d & acc = res[2];

    // Interpolate orientation
    Eigen::Matrix3d ori_target = oriSpline_.eval(currTime_);
    sva::PTransformd target(ori_target, pos);

    // Set the trajectory tracking task targets from the trajectory.
    Eigen::VectorXd refVel(6);
    Eigen::VectorXd refAcc(6);
    refVel.head<3>() = Eigen::Vector3d::Zero();
    refVel.tail<3>() = vel;
    refAcc.head<3>() = Eigen::Vector3d::Zero();
    refAcc.tail<3>() = acc;
    this->refVel(refVel);
    this->refAccel(refAcc);
    this->refPose(target);
    currTime_ = std::min(currTime_ + solver.dt(), duration_);
  }
  else
  {
    this->refVel(Eigen::Vector6d::Zero());
    this->refAccel(Eigen::Vector6d::Zero());
  }
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::interpolateGains()
{
  if(dimWeightInterpolator_.hasValues())
  {
    TrajectoryBase::dimWeight(dimWeightInterpolator_.compute(currTime_));
  }

  if(stiffnessInterpolator_.hasValues())
  { // Interpolate gains
    auto stiffness = stiffnessInterpolator_.compute(currTime_);
    if(dampingInterpolator_.hasValues())
    {
      TrajectoryBase::setGains(stiffness, dampingInterpolator_.compute(currTime_));
    }
    else
    { // damping is 2*sqrt(stiffness)
      TrajectoryBase::stiffness(stiffness);
    }
  }
  else if(dampingInterpolator_.hasValues())
  { // Interpolate only damping
    TrajectoryBase::damping(dampingInterpolator_.compute(currTime_));
  }
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::oriWaypoints(const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
{
  oriSpline_.waypoints(oriWp);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::dimWeight(const Eigen::VectorXd & dimW)
{
  if(dimW.size() != 6)
  {
    mc_rtc::log::error_and_throw("SplineTrajectoryTask dimWeight must be a Vector6d!");
  }

  dimWeightInterpolator_.clear();
  TrajectoryBase::dimWeight(dimW);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::stiffnessInterpolation(
    const std::vector<std::pair<double, Eigen::Vector6d>> & stiffnesses)
{
  stiffnessInterpolator_.values(stiffnesses);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::dampingInterpolation(const std::vector<std::pair<double, Eigen::Vector6d>> & damping)
{
  dampingInterpolator_.values(damping);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::dimWeightInterpolation(
    const std::vector<std::pair<double, Eigen::Vector6d>> & dimWeight)
{
  dimWeightInterpolator_.values(dimWeight);
}

template<typename Derived>
Eigen::VectorXd SplineTrajectoryTask<Derived>::dimWeight() const
{
  return TrajectoryBase::dimWeight();
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::stiffness(double stiffness)
{
  this->stiffness(stiffness * Eigen::Vector6d::Ones());
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::stiffness(const Eigen::VectorXd & stiffness)
{
  setGains(stiffness, stiffness.cwiseSqrt());
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::damping(double damping)
{
  this->damping(damping * Eigen::Vector6d::Ones());
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::damping(const Eigen::VectorXd & damping)
{
  if(damping.size() != 6)
  {
    mc_rtc::log::error_and_throw("[{}] dimensional damping must be a Vector6d!", name());
  }

  dampingInterpolator_.clear();
  TrajectoryBase::damping(damping);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::setGains(double stiffness, double damping)
{
  setGains(stiffness * Eigen::Vector6d::Ones(), damping * Eigen::Vector6d::Ones());
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::setGains(const Eigen::VectorXd & stiffness, const Eigen::VectorXd & damping)
{
  if(stiffness.size() != 6)
  {
    mc_rtc::log::error_and_throw("[{}] dimensional stiffness must be a Vector6d!", name());
  }
  else if(damping.size() != 6)
  {
    mc_rtc::log::error_and_throw("[{}] dimensional damping must be a Vector6d!", name());
  }
  stiffnessInterpolator_.clear();
  dampingInterpolator_.clear();
  TrajectoryBase::setGains(stiffness, damping);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::target(const sva::PTransformd & target)
{
  auto & derived = static_cast<Derived &>(*this);
  derived.targetPos(target.translation());
  oriSpline_.target(target.rotation());
}

template<typename Derived>
const sva::PTransformd SplineTrajectoryTask<Derived>::target() const
{
  const auto & derived = static_cast<const Derived &>(*this);
  return sva::PTransformd(oriSpline_.target(), derived.targetPos());
}

template<typename Derived>
Eigen::VectorXd SplineTrajectoryTask<Derived>::eval() const
{
  return sva::transformError(frame_->position(), target()).vector();
}

template<typename Derived>
Eigen::VectorXd SplineTrajectoryTask<Derived>::evalTracking() const
{
  return TrajectoryBase::eval();
}

template<typename Derived>
bool SplineTrajectoryTask<Derived>::timeElapsed() const
{
  return currTime_ >= duration_;
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::displaySamples(unsigned s)
{
  samples_ = s;
}

template<typename Derived>
unsigned SplineTrajectoryTask<Derived>::displaySamples() const
{
  return samples_;
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::refPose(const sva::PTransformd & target)
{
  errorT->target(target);
}

template<typename Derived>
const sva::PTransformd & SplineTrajectoryTask<Derived>::refPose() const
{
  return errorT->target();
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_surfacePose", this, [this]() { return frame_->position(); });
  MC_RTC_LOG_GETTER(name_ + "_targetPose", target);
  MC_RTC_LOG_GETTER(name_ + "_refPose", refPose);
  MC_RTC_LOG_HELPER(name_ + "_speed", speed);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTask::addToGUI(gui);

  gui.addElement({"Tasks", name_}, mc_rtc::gui::Checkbox(
                                       "Paused", [this]() { return paused_; }, [this]() { paused_ = !paused_; }));
  gui.addElement({"Tasks", name_}, mc_rtc::gui::Transform("Surface pose", [this]() { return frame_->position(); }));

  gui.addElement({"Tasks", name_}, mc_rtc::gui::Rotation(
                                       "Target Rotation", [this]() { return this->target(); },
                                       [this](const Eigen::Quaterniond & ori) {
                                         sva::PTransformd X_0_t(ori, this->target().translation());
                                         this->target(X_0_t);
                                       }));

  // Target rotation is handled independently
  auto & spline = static_cast<Derived &>(*this).spline();
  for(unsigned i = 0; i < oriSpline_.waypoints().size(); ++i)
  {
    gui.addElement(
        {"Tasks", name_, "Orientation Waypoint"},
        mc_rtc::gui::Rotation(
            "Waypoint " + std::to_string(i),
            [this, i, &spline]() {
              // Get position of orientation waypoint along the spline
              const auto & wp = this->oriSpline_.waypoint(i);
              return sva::PTransformd(wp.second, spline.splev(wp.first, 0)[0]);
            },
            [this, i](const Eigen::Quaterniond & ori) { this->oriSpline_.waypoint(i, ori.toRotationMatrix()); }));
  }
}

} // namespace mc_tasks
