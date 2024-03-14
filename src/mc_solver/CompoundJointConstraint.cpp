/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/CompoundJointConstraint.h>

#include <mc_rtc/logging.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TVMQPSolver.h>
#include <mc_solver/TasksQPSolver.h>

#include <mc_tvm/CompoundJointFunction.h>

namespace mc_solver
{

namespace details
{

CompoundJointConstraint::CompoundJointConstraint(const mc_rbdyn::Robots & robots, unsigned int rIndex, double dt)
: rIndex_(rIndex), name_("CompoundJointConstraint_" + robots.robot(rIndex).name()), dt_(dt)
{
}

CompoundJointConstraint::CompoundJointConstraint(const mc_rbdyn::Robots & robots,
                                                 unsigned int rIndex,
                                                 double dt,
                                                 const CompoundJointConstraintDescriptionVector & desc)
: rIndex_(rIndex), name_("CompoundJointConstraint_" + robots.robot(rIndex).name()), dt_(dt)
{
  for(const auto & d : desc) { addConstraint(robots, rIndex, d); }
}

CompoundJointConstraint::~CompoundJointConstraint() {}

void CompoundJointConstraint::addConstraint(const mc_rbdyn::Robots & robots,
                                            unsigned int rIndex,
                                            const CompoundJointConstraintDescription & desc)
{
  const auto & robot = robots.robot(rIndex);
  if(rIndex != rIndex_) { mc_rtc::log::error_and_throw("You must create one CompoundJointConstraint per robot"); }
  auto check_joint = [&](const std::string & jname)
  {
    if(!robot.hasJoint(jname)) { mc_rtc::log::error_and_throw("No joint named {} in {}", jname, robot.name()); }
    auto qIdx = robot.jointIndexByName(jname);
    if(robot.mb().joint(static_cast<int>(qIdx)).dof() != 1)
    {
      mc_rtc::log::error_and_throw("Joint {} does not have exactly one dof", jname);
    }
    return qIdx;
  };
  auto q1Idx = check_joint(desc.j1);
  auto q2Idx = check_joint(desc.j2);
  descs_.push_back({q1Idx, robot.mb().jointPosInDof(static_cast<int>(q1Idx)), q2Idx,
                    robot.mb().jointPosInDof(static_cast<int>(q2Idx)), desc.p1.x(), desc.p1.y(),
                    desc.p2.x() - desc.p1.x(), desc.p2.y() - desc.p1.y()});
}

void CompoundJointConstraint::updateNrVars(const std::vector<rbd::MultiBody> &, const tasks::qp::SolverData & data)
{
  A_.setZero(static_cast<int>(descs_.size()), data.nrVars());
  b_.setZero(static_cast<int>(descs_.size()));
  b_cst_.setZero(static_cast<int>(descs_.size()));
  auto ABegin = data.alphaDBegin(static_cast<int>(rIndex_));
  for(size_t i = 0; i < descs_.size(); ++i)
  {
    const auto & d = descs_[i];
    A_(static_cast<int>(i), ABegin + d.q1MatIdx) = dt_ * dt_ * d.P_y / 2;
    A_(static_cast<int>(i), ABegin + d.q2MatIdx) = -dt_ * dt_ * d.P_x / 2;
    b_cst_(static_cast<int>(i)) = d.p1_x * d.P_y - d.p1_y * d.P_x;
  }
}

void CompoundJointConstraint::update(const std::vector<rbd::MultiBody> &,
                                     const std::vector<rbd::MultiBodyConfig> & mbcs,
                                     const tasks::qp::SolverData &)
{
  for(size_t i = 0; i < descs_.size(); ++i)
  {
    const auto & d = descs_[i];
    const auto & q1 = mbcs[rIndex_].q[d.q1Idx][0];
    const auto & alpha1 = mbcs[rIndex_].alpha[d.q1Idx][0];
    const auto & q2 = mbcs[rIndex_].q[d.q2Idx][0];
    const auto & alpha2 = mbcs[rIndex_].alpha[d.q2Idx][0];
    b_(static_cast<int>(i)) =
        b_cst_(static_cast<int>(i)) + d.P_x * q2 - d.P_y * q1 + dt_ * (d.P_x * alpha2 - d.P_y * alpha1);
  }
}

std::string CompoundJointConstraint::descInEq(const std::vector<rbd::MultiBody> &, int i)
{
  std::stringstream ss;
  ss << "Error in " << name_ << " at line " << i << "\n";
  return ss.str();
}

struct TVMCompoundJointConstraint
{
  std::vector<mc_tvm::CompoundJointFunctionPtr> functions_;
  std::vector<tvm::TaskWithRequirementsPtr> constraints_;

  TVMCompoundJointConstraint(const mc_rbdyn::Robot & robot)
  {
    const auto & descs = robot.module().compoundJoints();
    functions_.reserve(descs.size());
    constraints_.reserve(functions_.size());
    for(const auto & cstr : descs)
    {
      functions_.push_back(std::make_shared<mc_tvm::CompoundJointFunction>(robot, cstr));
    }
  }

  void addToSolver(mc_solver::TVMQPSolver & solver)
  {
    for(const auto & f : functions_) { constraints_.push_back(solver.problem().add(f <= 0.)); }
  }

  void removeFromSolver(mc_solver::TVMQPSolver & solver)
  {
    for(const auto & c : constraints_) { solver.problem().remove(*c); }
    constraints_.clear();
  }
};

} // namespace details

static mc_rtc::void_ptr make_constraint(QPSolver::Backend backend,
                                        const mc_rbdyn::Robots & robots,
                                        unsigned int rIndex,
                                        double dt,
                                        const CompoundJointConstraintDescriptionVector & cs)
{
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
    {
      return mc_rtc::make_void_ptr<details::CompoundJointConstraint>(robots, rIndex, dt, cs);
    }
    case QPSolver::Backend::TVM:
    {
      return mc_rtc::make_void_ptr<details::TVMCompoundJointConstraint>(robots.robot(rIndex));
    }
    default:
      mc_rtc::log::error_and_throw("[CompoundJointConstraint] Not implemented for solver backend: {}", backend);
  }
}

CompoundJointConstraint::CompoundJointConstraint(const mc_rbdyn::Robots & robots, unsigned int rIndex, double dt)
: CompoundJointConstraint(robots, rIndex, dt, robots.robot(rIndex).module().compoundJoints())
{
}

CompoundJointConstraint::CompoundJointConstraint(const mc_rbdyn::Robots & robots,
                                                 unsigned int rIndex,
                                                 double dt,
                                                 const CompoundJointConstraintDescriptionVector & cs)
: constraint_(make_constraint(backend_, robots, rIndex, dt, cs))
{
}

void CompoundJointConstraint::addToSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      static_cast<details::CompoundJointConstraint *>(constraint_.get())
          ->addToSolver(solver.robots().mbs(), tasks_solver(solver).solver());
      break;
    case QPSolver::Backend::TVM:
      static_cast<details::TVMCompoundJointConstraint *>(constraint_.get())->addToSolver(tvm_solver(solver));
      break;
    default:
      break;
  }
}

void CompoundJointConstraint::removeFromSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      static_cast<details::CompoundJointConstraint *>(constraint_.get())
          ->removeFromSolver(tasks_solver(solver).solver());
      break;
    case QPSolver::Backend::TVM:
      static_cast<details::TVMCompoundJointConstraint *>(constraint_.get())->removeFromSolver(tvm_solver(solver));
      break;
    default:
      break;
  }
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintSetLoader::register_load_function(
    "compoundJoint",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      auto rIndex = robotIndexFromConfig(config, solver.robots(), "compoundJoint");
      if(config.has("constraints"))
      {
        return std::make_shared<mc_solver::CompoundJointConstraint>(solver.robots(), rIndex, solver.dt(),
                                                                    config("constraints"));
      }
      else { return std::make_shared<mc_solver::CompoundJointConstraint>(solver.robots(), rIndex, solver.dt()); }
    });
} // namespace
