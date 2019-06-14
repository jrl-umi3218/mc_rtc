#include <mc_rtc/logging.h>
#include <mc_trajectory/ExactCubic.h>

namespace mc_trajectory
{
using point_t = ExactCubic::point_t;
using exact_cubic_t = ExactCubic::exact_cubic_t;
using waypoint_t = ExactCubic::waypoint_t;
using spline_deriv_constraint_t = ExactCubic::spline_deriv_constraint_t;
using spline_constraints_t = ExactCubic::spline_constraints_t;

template<typename T>
std::string to_string_with_precision(const T a_value, const int n = 2)
{
  std::ostringstream out;
  out.precision(n);
  out << std::fixed << a_value;
  return out.str();
}

ExactCubic::ExactCubic(const std::vector<waypoint_t> & waypoints,
                       const point_t & init_vel,
                       const point_t & init_acc,
                       const point_t & end_vel,
                       const point_t & end_acc)
{
  constraints_.init_vel = init_vel;
  constraints_.init_acc = init_acc;
  constraints_.end_vel = end_vel;
  constraints_.end_acc = end_acc;
  this->waypoints(waypoints);
}

void ExactCubic::waypoints(const std::vector<waypoint_t> & waypoints)
{
  waypoints_ = waypoints;
  spline_.reset(new spline_deriv_constraint_t(waypoints_.begin(), waypoints_.end(), constraints_));
  needsUpdate_ = true;
}

void ExactCubic::waypoint(size_t idx, const point_t & waypoint)
{
  if(idx >= waypoints_.size())
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Cannot modify waypoint with index " << idx);
  }
  waypoints_[idx].second = waypoint;
  waypoints(waypoints_);
}

void ExactCubic::waypoint(size_t idx, double t)
{
  if(idx >= waypoints_.size())
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Cannot modify waypoint with index " << idx);
  }
  waypoints_[idx].first = t;
  waypoints(waypoints_);
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

void ExactCubic::target(const point_t & target)
{
  waypoints_.back().second = target;
  waypoints(waypoints_);
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
  std::vector<Eigen::Vector3d> traj;
  traj.resize(samples);
  // Evaluate trajectory for display
  for(unsigned i = 0; i < samples; ++i)
  {
    auto time = spline_->min() + (spline_->max() - spline_->min()) * i / (samples - 1);
    auto res = splev({time}, 0);
    Eigen::Vector3d & pos = res[0];
    traj[i] = pos;
  }
  return traj;
}

void ExactCubic::addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category)
{
  gui.addElement(category, mc_rtc::gui::Point3D("Target Position", [this]() { return target(); },
                                                [this](const Eigen::Vector3d & pos) { target(pos); }));

  samples_ = sampleTrajectory(samplingPoints_);
  gui.addElement(category, mc_rtc::gui::Trajectory("Trajectory", [this]() {
                   if(this->needsUpdate_)
                   {
                     samples_ = this->sampleTrajectory(samplingPoints_);
                     needsUpdate_ = false;
                   }
                   return samples_;
                 }));

  // Interactive control points (target is handled independently)
  std::vector<std::string> waypointCategory = category;
  waypointCategory.push_back("Position Control Points");
  for(unsigned int i = 0; i < waypoints_.size() - 1; ++i)
  {
    gui.addElement(waypointCategory,
                   mc_rtc::gui::Point3D("Waypoint " + std::to_string(i), [this, i]() { return waypoint(i).second; },
                                        [this, i](const Eigen::Vector3d & pos) { waypoint(i, pos); }),
                   mc_rtc::gui::NumberInput("Time " + std::to_string(i), [this, i]() { return waypointTime(i); },
                                            [this, i](double t) { waypoint(i, t); }));
  }
}

} // namespace mc_trajectory
