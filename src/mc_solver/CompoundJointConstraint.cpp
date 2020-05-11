#include <mc_rtc/logging.h>
#include <mc_solver/CompoundJointConstraint.h>
#include <mc_solver/ConstraintSetLoader.h>

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
  for(const auto & d : desc)
  {
    addConstraint(robots, rIndex, d);
  }
}

CompoundJointConstraint::~CompoundJointConstraint() {}

void CompoundJointConstraint::addConstraint(const mc_rbdyn::Robots & robots,
                                            unsigned int rIndex,
                                            const CompoundJointConstraintDescription & desc)
{
  const auto & robot = robots.robot(rIndex);
  if(rIndex != rIndex_)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "You must create one CompoundJointConstraint per robot");
  }
  auto check_joint = [&](const std::string & jname) {
    if(!robot.hasJoint(jname))
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "No joint named " << jname << " in " << robot.name())
    }
    auto qIdx = robot.jointIndexByName(jname);
    if(robot.mb().joint(static_cast<int>(qIdx)).dof() != 1)
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "Joint " << jname << " does not have exactly one dof")
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

} // namespace details

CompoundJointConstraint::CompoundJointConstraint(const mc_rbdyn::Robots & robots, unsigned int rIndex, double dt)
: constr_(robots, rIndex, dt, robots.robot(rIndex).module().compoundJoints())
{
}

void CompoundJointConstraint::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver)
{
  constr_.addToSolver(mbs, solver);
  solver.updateConstrSize();
}

void CompoundJointConstraint::removeFromSolver(tasks::qp::QPSolver & solver)
{
  constr_.removeFromSolver(solver);
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintSetLoader::register_load_function(
    "compoundJoint",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      return std::make_shared<mc_solver::CompoundJointConstraint>(
          solver.robots(), robotIndexFromConfig(config, solver.robots(), "compoundJoint"), solver.dt());
    });
}
