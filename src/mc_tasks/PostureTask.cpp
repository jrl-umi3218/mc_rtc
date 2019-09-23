/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/PostureTask.h>

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

namespace mc_tasks
{

PostureTask::PostureTask(const mc_solver::QPSolver & solver, unsigned int rIndex, double stiffness, double weight)
: robots_(solver.robots()), rIndex_(rIndex),
  pt_(solver.robots().mbs(), rIndex_, robots_.robot(rIndex_).mbc().q, stiffness, weight), dt_(solver.dt()),
  eval_(pt_.eval()), speed_(pt_.eval())
{
  eval_.setZero();
  speed_.setZero();
  type_ = "posture";
  name_ = std::string("posture_") + robots_.robot(rIndex_).name();
  posture_ = pt_.posture();
  for(const auto & j : robots_.robot(rIndex_).mb().joints())
  {
    if(j.isMimic())
    {
      mimics_[j.mimicName()].push_back(robots_.robot(rIndex_).jointIndexByName(j.name()));
    }
  }
}

void PostureTask::reset()
{
  pt_.posture(robots_.robot(rIndex_).mbc().q);
  posture_ = pt_.posture();
}

void PostureTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                     const std::vector<std::string> & activeJointsName,
                                     const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  std::vector<std::string> unactiveJoints = {};
  for(const auto & j : robots_.robot(rIndex_).mb().joints())
  {
    if(j.dof() && std::find(activeJointsName.begin(), activeJointsName.end(), j.name()) == activeJointsName.end())
    {
      unactiveJoints.push_back(j.name());
    }
  }
  selectUnactiveJoints(solver, unactiveJoints);
}

void PostureTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                       const std::vector<std::string> & unactiveJointsName,
                                       const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  std::vector<tasks::qp::JointStiffness> jsv;
  for(const auto & j : unactiveJointsName)
  {
    jsv.emplace_back(j, 0.0);
  }
  pt_.jointsStiffness(solver.robots().mbs(), jsv);
}

void PostureTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  pt_.jointsStiffness(solver.robots().mbs(), {});
}

Eigen::VectorXd PostureTask::eval() const
{
  return pt_.eval();
}

Eigen::VectorXd PostureTask::speed() const
{
  return speed_;
}

void PostureTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver_)
  {
    solver.addTask(&pt_);
    inSolver_ = true;
  }
}

void PostureTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(inSolver_)
  {
    solver.removeTask(&pt_);
    inSolver_ = false;
  }
}

void PostureTask::update()
{
  speed_ = (pt_.eval() - eval_) / dt_;
  eval_ = pt_.eval();
}

void PostureTask::posture(const std::vector<std::vector<double>> & p)
{
  pt_.posture(p);
  posture_ = p;
}

std::vector<std::vector<double>> PostureTask::posture() const
{
  return pt_.posture();
}

void PostureTask::stiffness(double s)
{
  pt_.stiffness(s);
}

double PostureTask::stiffness() const
{
  return pt_.stiffness();
}

void PostureTask::weight(double w)
{
  pt_.weight(w);
}

double PostureTask::weight() const
{
  return pt_.weight();
}

bool PostureTask::inSolver() const
{
  return inSolver_;
}

void PostureTask::jointGains(const mc_solver::QPSolver & solver, const std::vector<tasks::qp::JointGains> & jgs)
{
  pt_.jointsGains(solver.robots().mbs(), jgs);
}

void PostureTask::jointStiffness(const mc_solver::QPSolver & solver, const std::vector<tasks::qp::JointStiffness> & jss)
{
  pt_.jointsStiffness(solver.robots().mbs(), jss);
}

void PostureTask::target(const std::map<std::string, std::vector<double>> & joints)
{
  auto q = posture();
  for(const auto & j : joints)
  {
    if(robots_.robot(rIndex_).hasJoint(j.first))
    {
      if(robots_.robot(rIndex_).mb().joint(robots_.robot(rIndex_).jointIndexByName(j.first)).dof() == j.second.size())
      {
        q[robots_.robot(rIndex_).jointIndexByName(j.first)] = j.second;
      }
      else
      {
        LOG_ERROR("PostureTask::target dof missmatch for " << j.first)
      }
    }
  }
  posture(q);
}

void PostureTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement({"Tasks", name_, "Gains"},
                 mc_rtc::gui::NumberInput("stiffness", [this]() { return this->stiffness(); },
                                          [this](const double & s) { this->stiffness(s); }),
                 mc_rtc::gui::NumberInput("weight", [this]() { return this->weight(); },
                                          [this](const double & w) { this->weight(w); }));
  for(const auto & j : robots_.robot(rIndex_).mb().joints())
  {
    if(j.dof() != 1 || j.isMimic())
    {
      continue;
    }
    auto jIndex = robots_.robot(rIndex_).jointIndexByName(j.name());
    bool isContinuous = robots_.robot(rIndex_).ql()[jIndex][0] == -std::numeric_limits<double>::infinity();
    auto updatePosture = [this](unsigned int jIndex, double v) {
      this->posture_[jIndex][0] = v;
      const auto & jName = robots_.robot(rIndex_).mb().joint(jIndex).name();
      if(mimics_.count(jName))
      {
        for(auto ji : mimics_.at(jName))
        {
          const auto & mimic = robots_.robot(rIndex_).mb().joint(ji);
          this->posture_[ji][0] = mimic.mimicMultiplier() * v + mimic.mimicOffset();
        }
      }
      posture(posture_);
    };
    if(isContinuous)
    {
      gui.addElement({"Tasks", name_, "Target"},
                     mc_rtc::gui::NumberInput(j.name(), [this, jIndex]() { return this->posture_[jIndex][0]; },
                                              [this, jIndex, updatePosture](double v) { updatePosture(jIndex, v); }));
    }
    else
    {
      gui.addElement({"Tasks", name_, "Target"},
                     mc_rtc::gui::NumberSlider(j.name(), [this, jIndex]() { return this->posture_[jIndex][0]; },
                                               [this, jIndex, updatePosture](double v) { updatePosture(jIndex, v); },
                                               robots_.robot(rIndex_).ql()[jIndex][0],
                                               robots_.robot(rIndex_).qu()[jIndex][0]));
    }
  }
}

} // namespace mc_tasks

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function(
    "posture",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t =
          std::make_shared<mc_tasks::PostureTask>(solver, config("robotIndex"), config("stiffness"), config("weight"));
      t->load(solver, config);
      if(config.has("posture"))
      {
        t->posture(config("posture"));
      }
      if(config.has("jointGains"))
      {
        t->jointGains(solver, config("jointGains"));
      }
      if(config.has("target"))
      {
        t->target(config("target"));
      }
      return t;
    });
}
