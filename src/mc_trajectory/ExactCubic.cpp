#include <mc_rtc/logging.h>
#include <mc_trajectory/ExactCubic.h>

namespace mc_trajectory
{
using point_t = ExactCubic::point_t;
using exact_cubic_t = ExactCubic::exact_cubic_t;
using waypoint_t = ExactCubic::waypoint_t;
using spline_deriv_constraint_t = ExactCubic::spline_deriv_constraint_t;
using spline_constraints_t = ExactCubic::spline_constraints_t;

ExactCubic::ExactCubic(const point_t & init_vel,
                       const point_t & init_acc,
                       const point_t & end_vel,
                       const point_t & end_acc)
{
  constraints(init_vel, init_acc, end_vel, end_acc);
}

ExactCubic::ExactCubic(const std::vector<waypoint_t> & waypoints,
                       const point_t & init_vel,
                       const point_t & init_acc,
                       const point_t & end_vel,
                       const point_t & end_acc)
{
  constraints(init_vel, init_acc, end_vel, end_acc);
  this->waypoints(waypoints);
}

void ExactCubic::update()
{
  if(needsUpdate_)
  {
    spline_.reset(new spline_deriv_constraint_t(waypoints_.begin(), waypoints_.end(), constraints_));
    samples_ = this->sampleTrajectory(samplingPoints_);
    needsUpdate_ = false;
  }
}

void ExactCubic::waypoints(const std::vector<waypoint_t> & waypoints)
{
  if(waypoints.size() < 2)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "ExactCubic splines should have at least 2 waypoints");
  }
  waypoints_ = waypoints;
  needsUpdate_ = true;
}

void ExactCubic::waypoint(size_t idx, const point_t & waypoint)
{
  if(idx >= waypoints_.size())
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Cannot modify waypoint with index " << idx);
  }
  waypoints_[idx].second = waypoint;
  needsUpdate_ = true;
}

void ExactCubic::waypoint(size_t idx, double t)
{
  if(idx >= waypoints_.size())
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Cannot modify waypoint with index " << idx);
  }
  waypoints_[idx].first = t;
  needsUpdate_ = true;
}

double ExactCubic::waypointTime(size_t idx) const
{
  if(idx >= waypoints_.size())
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "No waypoint with index " << idx);
  }
  return waypoints_[idx].first;
}

const std::vector<waypoint_t> & ExactCubic::waypoints() const
{
  return waypoints_;
}

const waypoint_t & ExactCubic::waypoint(size_t idx) const
{
  if(idx >= waypoints_.size())
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "No waypoint with index " << idx);
  }
  return waypoints_[idx];
}

void ExactCubic::constraints(const point_t & init_vel,
                             const point_t & init_acc,
                             const point_t & end_vel,
                             const point_t & end_acc)
{
  constraints_.init_vel = init_vel;
  constraints_.init_acc = init_acc;
  constraints_.end_vel = end_vel;
  constraints_.end_acc = end_acc;
  needsUpdate_ = true;
}

void ExactCubic::target(const point_t & target)
{
  waypoints_.back().second = target;
  needsUpdate_ = true;
}

const point_t & ExactCubic::target() const
{
  return waypoints_.back().second;
}

const point_t & ExactCubic::init_vel() const
{
  return constraints_.init_vel;
}
const point_t & ExactCubic::init_acc() const
{
  return constraints_.init_acc;
}
const point_t & ExactCubic::end_vel() const
{
  return constraints_.end_vel;
}
const point_t & ExactCubic::end_acc() const
{
  return constraints_.end_acc;
}

void ExactCubic::samplingPoints(const unsigned s)
{
  if(s != samplingPoints_)
  {
    needsUpdate_ = true;
  }
  samplingPoints_ = s;
}

unsigned ExactCubic::samplingPoints() const
{
  return samplingPoints_;
}

std::vector<point_t> ExactCubic::splev(double t, unsigned int der)
{
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(der + 1);
  for(std::size_t order = 0; order <= der; ++order)
  {
    pts.push_back(spline_->derivate(t, order));
  }
  return pts;
}

std::vector<Eigen::Vector3d> ExactCubic::sampleTrajectory(unsigned samples)
{
  if(samples < 1)
  {
    LOG_ERROR("There should be at least 1 sample");
    return {};
  }
  std::vector<Eigen::Vector3d> traj;
  traj.resize(samples);
  // Evaluate trajectory for display
  for(unsigned i = 0; i < samples; ++i)
  {
    auto time = spline_->min() + (spline_->max() - spline_->min()) * i / (samples - 1);
    auto res = splev(time, 0);
    traj[i] = res[0];
  }
  return traj;
}

void ExactCubic::addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category)
{
  gui.addElement(category,
                 mc_rtc::gui::Point3D("Target Position", [this]() -> const Eigen::Vector3d & { return target(); },
                                      [this](const Eigen::Vector3d & pos) { target(pos); }));

  gui.addElement(category, mc_rtc::gui::Trajectory("Trajectory", [this]() { return samples_; }));

  // Interactive control points (target is handled independently)
  std::vector<std::string> waypointCategory = category;
  waypointCategory.push_back("Position Control Points");
  for(unsigned int i = 0; i < waypoints_.size() - 1; ++i)
  {
    gui.addElement(waypointCategory,
                   mc_rtc::gui::Point3D("Waypoint " + std::to_string(i),
                                        [this, i]() -> const Eigen::Vector3d & { return waypoint(i).second; },
                                        [this, i](const Eigen::Vector3d & pos) { waypoint(i, pos); }),
                   mc_rtc::gui::NumberInput("Time " + std::to_string(i), [this, i]() { return waypointTime(i); },
                                            [this, i](double t) { waypoint(i, t); }));
  }
}

} // namespace mc_trajectory
