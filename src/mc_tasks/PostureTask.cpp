/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/PostureTask.h>

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_solver/TVMQPSolver.h>
#include <mc_solver/TasksQPSolver.h>

#include <mc_tvm/PostureFunction.h>
#include <mc_tvm/Robot.h>

#include <mc_rbdyn/configuration_io.h>

#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/NumberSlider.h>

namespace mc_tasks
{

namespace details
{

inline static mc_rtc::void_ptr_caster<mc_tvm::PostureFunction> tvm_error{};

struct TVMPostureTask : public TrajectoryTaskGeneric
{
  TVMPostureTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
  : TrajectoryTaskGeneric(robots, robotIndex, stiffness, weight)
  {
    finalize<Backend::TVM, mc_tvm::PostureFunction>(robots.robot(robotIndex));
    type_ = "posture";
    name_ = std::string("posture_") + robots.robot(robotIndex).name();
  }

  void update(mc_solver::QPSolver & solver) override { TrajectoryTaskGeneric::update(solver); }

  void posture(const std::vector<std::vector<double>> & p) { tvm_error(errorT)->posture(p); }

  void jointsGains(const std::vector<tasks::qp::JointGains> & gains)
  {
    const auto & robot = robots.robot(rIndex);
    int offset = robot.tvmRobot().qFloatingBase()->size() != 0 ? -6 : 0;
    for(const auto & g : gains)
    {
      auto mbcIdx = robot.mb().jointIndexByName(g.jointName);
      if(mbcIdx < 0)
      {
        mc_rtc::log::error("[PostureTask::jointGains] {} is not in this robot", g.jointName);
        continue;
      }
      auto jIdx = robot.mb().jointPosInDof(mbcIdx) + offset;
      stiffness_(jIdx) = g.stiffness;
      damping_(jIdx) = g.damping;
    }
    setGains(stiffness_, damping_);
  }

  void jointsStiffness(const std::vector<tasks::qp::JointStiffness> & gains)
  {
    const auto & robot = robots.robot(rIndex);
    int offset = robot.tvmRobot().qFloatingBase()->size() != 0 ? -6 : 0;
    for(const auto & g : gains)
    {
      auto mbcIdx = robot.mb().jointIndexByName(g.jointName);
      if(mbcIdx < 0)
      {
        mc_rtc::log::error("[PostureTask::jointGains] {} is not in this robot", g.jointName);
        continue;
      }
      auto jIdx = robot.mb().jointPosInDof(mbcIdx) + offset;
      stiffness_(jIdx) = g.stiffness;
      damping_(jIdx) = 2 * sqrt(g.stiffness);
    }
    setGains(stiffness_, damping_);
  }
};

} // namespace details

inline static mc_rtc::void_ptr_caster<tasks::qp::PostureTask> tasks_error{};
inline static mc_rtc::void_ptr_caster<details::TVMPostureTask> tvm_error{};

inline static mc_rtc::void_ptr make_error(MetaTask::Backend backend,
                                          const mc_solver::QPSolver & solver,
                                          unsigned int rIndex,
                                          double stiffness,
                                          double weight)
{
  switch(backend)
  {
    case MetaTask::Backend::Tasks:
      return mc_rtc::make_void_ptr<tasks::qp::PostureTask>(solver.robots().mbs(), static_cast<int>(rIndex),
                                                           solver.robot(rIndex).mbc().q, stiffness, weight);
    case MetaTask::Backend::TVM:
      return mc_rtc::make_void_ptr<details::TVMPostureTask>(solver.robots(), rIndex, stiffness, weight);
    default:
      mc_rtc::log::error_and_throw("[PosutreTask] Not implemented for solver backend: {}", backend);
  }
}

PostureTask::PostureTask(const mc_solver::QPSolver & solver, unsigned int rIndex, double stiffness, double weight)
: robots_(solver.robots()), rIndex_(rIndex), pt_(make_error(backend_, solver, rIndex, stiffness, weight)),
  dt_(solver.dt())
{
  eval_ = this->eval();
  speed_ = Eigen::VectorXd::Zero(eval_.size());
  type_ = "posture";
  name_ = std::string("posture_") + robots_.robot(rIndex_).name();
  for(const auto & j : robots_.robot(rIndex_).mb().joints())
  {
    if(j.isMimic())
    {
      mimics_[j.mimicName()].push_back(static_cast<int>(robots_.robot(rIndex_).jointIndexByName(j.name())));
    }
  }
  reset();
}

void PostureTask::reset()
{
  posture(robots_.robot(rIndex_).mbc().q);
}

void PostureTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  MetaTask::load(solver, config);
  if(config.has("posture")) { this->posture(config("posture")); }
  if(config.has("jointGains")) { this->jointGains(solver, config("jointGains")); }
  if(config.has("target")) { this->target(config("target")); }
  if(config.has("stiffness")) { this->stiffness(config("stiffness")); }
  if(config.has("weight")) { this->weight(config("weight")); }
  if(config.has("jointWeights")) { this->jointWeights(config("jointWeights")); }
}

void PostureTask::dimWeight(const Eigen::VectorXd & dimW)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(pt_)->dimWeight(dimW);
      break;
    case Backend::TVM:
      tvm_error(pt_)->dimWeight(dimW);
      break;
    default:
      break;
  }
}

Eigen::VectorXd PostureTask::dimWeight() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_error(pt_)->dimWeight();
    case Backend::TVM:
      return tvm_error(pt_)->dimWeight();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void PostureTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                     const std::vector<std::string> & activeJointsName,
                                     const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  ensureHasJoints(robots_.robot(rIndex_), activeJointsName, "[" + name() + "::selectActiveJoints]");
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

void PostureTask::selectUnactiveJoints(mc_solver::QPSolver &,
                                       const std::vector<std::string> & unactiveJointsName,
                                       const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  ensureHasJoints(robots_.robot(rIndex_), unactiveJointsName, "[" + name() + "::selectUnActiveJoints]");
  Eigen::VectorXd dimW = dimWeight();
  dimW.setOnes();
  const auto & robot = robots_.robot(rIndex_);
  auto dofOffset = [&robot, this]()
  {
    switch(backend_)
    {
      case Backend::Tasks:
        return 0;
      case Backend::TVM:
        return robot.mb().joint(0).dof();
      default:
        mc_rtc::log::error_and_throw("Not implemented in backend {}", backend_);
    }
  }();
  for(const auto & j : unactiveJointsName)
  {
    auto jIndex = static_cast<int>(robot.jointIndexByName(j));
    const auto & joint = robot.mb().joint(jIndex);
    if(joint.dof() == 6) { continue; }
    auto dofIndex = robot.mb().jointPosInDof(jIndex) - dofOffset;
    dimW.segment(dofIndex, joint.dof()).setZero();
  }
  dimWeight(dimW);
}

void PostureTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  selectUnactiveJoints(solver, {});
}

Eigen::VectorXd PostureTask::eval() const
{
  switch(backend_)
  {
    case Backend::Tasks:
    {
      auto & pt = *tasks_error(pt_);
      return pt.dimWeight().asDiagonal() * pt.eval();
    }
    case Backend::TVM:
      return tvm_error(pt_)->eval();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

Eigen::VectorXd PostureTask::speed() const
{
  return speed_;
}

void PostureTask::refVel(const Eigen::VectorXd & refVel) noexcept
{
  assert(refVel.size() == robots_.robot(rIndex_).mb().nrDof());
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(pt_)->refVel(refVel);
      break;
    case Backend::TVM:
      tvm_error(pt_)->refVel(refVel);
      break;
    default:
      break;
  }
}

const Eigen::VectorXd & PostureTask::refVel() const noexcept
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_error(pt_)->refVel();
    case Backend::TVM:
      return tvm_error(pt_)->refVel();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void PostureTask::refAccel(const Eigen::VectorXd & refAccel) noexcept
{
  assert(refAccel.size() == robots_.robot(rIndex_).mb().nrDof());
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(pt_)->refAccel(refAccel);
      break;
    case Backend::TVM:
      tvm_error(pt_)->refAccel(refAccel);
      break;
    default:
      break;
  }
}

const Eigen::VectorXd & PostureTask::refAccel() const noexcept
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_error(pt_)->refAccel();
    case Backend::TVM:
      return tvm_error(pt_)->refAccel();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void PostureTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(inSolver_) { return; }
  inSolver_ = true;
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_solver(solver).addTask(tasks_error(pt_));
      break;
    case Backend::TVM:
      MetaTask::addToSolver(*tvm_error(pt_), solver);
      break;
    default:
      break;
  }
}

void PostureTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver_) { return; }
  inSolver_ = false;
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_solver(solver).removeTask(tasks_error(pt_));
      break;
    case Backend::TVM:
      MetaTask::removeFromSolver(*tvm_error(pt_), solver);
      break;
    default:
      break;
  }
}

void PostureTask::update(mc_solver::QPSolver & solver)
{
  switch(backend_)
  {
    case Backend::Tasks:
    {
      const auto & pt = *tasks_error(pt_);
      speed_ = pt.dimWeight().asDiagonal() * (pt.eval() - eval_) / dt_;
      eval_ = pt.eval();
      break;
    }
    case Backend::TVM:
    {
      auto & pt = *tvm_error(pt_);
      pt.update(solver);
      speed_ = (pt.eval() - eval_) / dt_;
      eval_ = pt.dimWeight().asDiagonal() * pt.eval();
      break;
    }
    default:
      break;
  }
}

void PostureTask::posture(const std::vector<std::vector<double>> & p)
{
  posture_ = p;
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(pt_)->posture(p);
      break;
    case Backend::TVM:
      tvm_error(pt_)->posture(p);
      break;
    default:
      break;
  }
}

std::vector<std::vector<double>> PostureTask::posture() const
{
  return posture_;
}

void PostureTask::stiffness(double s)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(pt_)->stiffness(s);
      break;
    case Backend::TVM:
      tvm_error(pt_)->stiffness(s);
      break;
    default:
      break;
  }
}

double PostureTask::stiffness() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_error(pt_)->stiffness();
    case Backend::TVM:
      return tvm_error(pt_)->stiffness();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void PostureTask::damping(double d)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(pt_)->gains(stiffness(), d);
      break;
    case Backend::TVM:
      tvm_error(pt_)->damping(d);
    default:
      break;
  }
}

double PostureTask::damping() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_error(pt_)->damping();
    case Backend::TVM:
      return tvm_error(pt_)->damping();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void PostureTask::setGains(double s, double d)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(pt_)->gains(s, d);
      break;
    case Backend::TVM:
      tvm_error(pt_)->setGains(s, d);
      break;
    default:
      break;
  }
}

void PostureTask::weight(double w)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(pt_)->weight(w);
      break;
    case Backend::TVM:
      tvm_error(pt_)->weight(w);
      break;
    default:
      break;
  }
}

double PostureTask::weight() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_error(pt_)->weight();
    case Backend::TVM:
      return tvm_error(pt_)->weight();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

bool PostureTask::inSolver() const
{
  return inSolver_;
}

void PostureTask::jointGains(const mc_solver::QPSolver & solver, const std::vector<tasks::qp::JointGains> & jgs)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(pt_)->jointsGains(solver.robots().mbs(), jgs);
      break;
    case Backend::TVM:
      tvm_error(pt_)->jointsGains(jgs);
      break;
    default:
      break;
  }
}

void PostureTask::jointStiffness(const mc_solver::QPSolver & solver, const std::vector<tasks::qp::JointStiffness> & jss)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(pt_)->jointsStiffness(solver.robots().mbs(), jss);
      break;
    case Backend::TVM:
      tvm_error(pt_)->jointsStiffness(jss);
      break;
    default:
      break;
  }
}

void PostureTask::jointWeights(const std::map<std::string, double> & jws)
{
  Eigen::VectorXd dimW = dimWeight();
  const auto & robot = robots_.robot(rIndex_);
  const auto & mb = robot.mb();
  for(const auto & jw : jws)
  {
    if(robot.hasJoint(jw.first))
    {
      auto jIndex = mb.jointIndexByName(jw.first);
      if(mb.joint(jIndex).dof() > 0) { dimW[mb.jointPosInDof(jIndex)] = jw.second; }
      // No warning, it's probably over specified
    }
    else
    {
      mc_rtc::log::warning("[PostureTask] No joint named {} in {}, joint weight will have no effect", jw.first,
                           robot.name());
    }
  }
  dimWeight(dimW);
}

void PostureTask::target(const std::map<std::string, std::vector<double>> & joints)
{
  auto q = posture();
  for(const auto & j : joints)
  {
    if(robots_.robot(rIndex_).hasJoint(j.first))
    {
      if(static_cast<size_t>(
             robots_.robot(rIndex_).mb().joint(static_cast<int>(robots_.robot(rIndex_).jointIndexByName(j.first))).dof())
         == j.second.size())
      {
        q[robots_.robot(rIndex_).jointIndexByName(j.first)] = j.second;
        if(mimics_.count(j.first))
        {
          for(auto ji : mimics_.at(j.first))
          {
            const auto & mimic = robots_.robot(rIndex_).mb().joint(ji);
            if(static_cast<size_t>(mimic.dof()) == j.second.size())
            {
              for(unsigned i = 0; i < j.second.size(); i++)
              {
                q[static_cast<size_t>(ji)][i] = mimic.mimicMultiplier() * j.second[i] + mimic.mimicOffset();
              }
            }
          }
        }
      }
      else { mc_rtc::log::error("PostureTask::target dof missmatch for {}", j.first); }
    }
  }
  posture(q);
}

void PostureTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_eval", this, [this]() { return eval(); });
  logger.addLogEntry(name_ + "_speed", this, [this]() -> const Eigen::VectorXd & { return speed_; });
  logger.addLogEntry(name_ + "_refVel", this, [this]() -> const Eigen::VectorXd & { return refVel(); });
  logger.addLogEntry(name_ + "_refAccel", this, [this]() -> const Eigen::VectorXd & { return refAccel(); });
}

void PostureTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement({"Tasks", name_, "Gains"},
                 mc_rtc::gui::NumberInput(
                     "stiffness", [this]() { return this->stiffness(); },
                     [this](const double & s) { this->setGains(s, this->damping()); }),
                 mc_rtc::gui::NumberInput(
                     "damping", [this]() { return this->damping(); },
                     [this](const double & d) { this->setGains(this->stiffness(), d); }),
                 mc_rtc::gui::NumberInput(
                     "stiffness & damping", [this]() { return this->stiffness(); },
                     [this](const double & g) { this->stiffness(g); }),
                 mc_rtc::gui::NumberInput(
                     "weight", [this]() { return this->weight(); }, [this](const double & w) { this->weight(w); }));
  std::vector<std::string> active_gripper_joints;
  for(const auto & g : robots_.robot(rIndex_).grippers())
  {
    for(const auto & n : g.get().activeJoints()) { active_gripper_joints.push_back(n); }
  }
  auto isActiveGripperJoint = [&](const std::string & j)
  { return std::find(active_gripper_joints.begin(), active_gripper_joints.end(), j) != active_gripper_joints.end(); };
  for(const auto & j : robots_.robot(rIndex_).mb().joints())
  {
    if(j.dof() != 1 || j.isMimic() || isActiveGripperJoint(j.name())) { continue; }
    auto jIndex = robots_.robot(rIndex_).jointIndexByName(j.name());
    bool isContinuous = robots_.robot(rIndex_).ql()[jIndex][0] == -std::numeric_limits<double>::infinity();
    auto updatePosture = [this](unsigned int jIndex, double v)
    {
      this->posture_[jIndex][0] = v;
      const auto & jName = robots_.robot(rIndex_).mb().joint(static_cast<int>(jIndex)).name();
      if(mimics_.count(jName))
      {
        for(auto ji : mimics_.at(jName))
        {
          const auto & mimic = robots_.robot(rIndex_).mb().joint(ji);
          this->posture_[static_cast<size_t>(ji)][0] = mimic.mimicMultiplier() * v + mimic.mimicOffset();
        }
      }
      posture(posture_);
    };
    if(isContinuous)
    {
      gui.addElement({"Tasks", name_, "Target"}, mc_rtc::gui::NumberInput(
                                                     j.name(), [this, jIndex]() { return this->posture_[jIndex][0]; },
                                                     [jIndex, updatePosture](double v) { updatePosture(jIndex, v); }));
    }
    else
    {
      gui.addElement({"Tasks", name_, "Target"},
                     mc_rtc::gui::NumberSlider(
                         j.name(), [this, jIndex]() { return this->posture_[jIndex][0]; },
                         [jIndex, updatePosture](double v) { updatePosture(jIndex, v); },
                         robots_.robot(rIndex_).ql()[jIndex][0], robots_.robot(rIndex_).qu()[jIndex][0]));
    }
  }
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "posture",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "posture");
      auto t =
          std::make_shared<mc_tasks::PostureTask>(solver, robotIndex, config("stiffness", 1.), config("weight", 10.));
      t->load(solver, config);
      return t;
    });
} // namespace
