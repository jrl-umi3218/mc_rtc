#include <mc_solver/SSMPFLConstraint.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TVMQPSolver.h>

#include <mc_rbdyn/Robot.h>
#include <mc_rtc/logging.h>

#include <tvm/Space.h>
#include <tvm/task_dynamics/None.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mc_solver
{

SSMPFLConstraint::SSMPFLConstraint(const mc_rbdyn::Robot & robot,
                                   unsigned int capacity,
                                   double dt,
                                   std::map<std::string, RegionParameters> regions,
                                   double stoppingTime,
                                   double minMobility,
                                   double minMassEigenvalue,
                                   double weight)
: weight_(weight),
  slack_(weight > 0 ? tvm::Space(static_cast<int>(capacity)).createVariable("SSMPFL row slack") : nullptr),
  function_(std::make_shared<mc_tvm::SSMPFLFunction>(robot,
                                                     capacity,
                                                     dt,
                                                     std::move(regions),
                                                     stoppingTime,
                                                     minMobility,
                                                     minMassEigenvalue,
                                                     slack_))
{
  if(backend_ != QPSolver::Backend::TVM) { throw std::invalid_argument("SSMPFLConstraint requires TVM backend"); }
  if(!std::isfinite(weight_) || weight_ < 0) { throw std::invalid_argument("invalid SSM+PFL weight"); }
}

void SSMPFLConstraint::setMeasurements(const std::vector<Measurement> & measurements)
{ function_->setMeasurements(measurements); }

void SSMPFLConstraint::clear()
{ function_->clear(); }

double SSMPFLConstraint::softSlack(size_t row) const
{
  if(row >= capacity()) { throw std::out_of_range("SSM+PFL row index"); }
  if(row >= function_->rowData().size()) { return 0.0; }
  return slack_ ? std::max(0.0, slack_->value()[static_cast<int>(row)]) : 0.0;
}

void SSMPFLConstraint::addToSolverImpl(QPSolver & solver)
{
  auto & problem = TVMQPSolver::from_solver(solver).problem();
  task_ = problem.add(function_ <= 0., tvm::task_dynamics::None(), {tvm::requirements::PriorityLevel(0)});
  if(slack_)
  {
    slackBound_ = problem.add(slack_ >= 0., {tvm::requirements::PriorityLevel(0)});
    slackObjective_ =
        problem.add(slack_ == 0., {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(weight_)});
  }
}

void SSMPFLConstraint::removeFromSolverImpl(QPSolver & solver)
{
  auto & problem = TVMQPSolver::from_solver(solver).problem();
  if(slackObjective_)
  {
    problem.remove(*slackObjective_);
    slackObjective_.reset();
  }
  if(slackBound_)
  {
    problem.remove(*slackBound_);
    slackBound_.reset();
  }
  if(task_)
  {
    problem.remove(*task_);
    task_.reset();
  }
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintSetLoader::register_load_function(
    "ssmPfl",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      if(solver.backend() != mc_solver::QPSolver::Backend::TVM)
        mc_rtc::log::error_and_throw("SSM+PFL requires the TVM solver backend");
      const unsigned int robotIndex =
          robotIndexFromConfig(config, solver.robots(), "ssmPfl", false, "robotIndex", "robot", "");
      const int capacity = config("capacity");
      const double stoppingTime = config("StoppingTimeSeconds");
      const double minMobility = config("MinDirectionalInverseMass", 1e-12);
      const double minMassEigenvalue = config("MinMassEigenvalue", 1e-9);
      const double weight = config("Weight", 0.0);
      if(capacity <= 0) { mc_rtc::log::error_and_throw("SSM+PFL capacity must be positive"); }
      if(config.has("Weight") && (!std::isfinite(weight) || weight <= 0))
        mc_rtc::log::error_and_throw("SSM+PFL Weight must be positive and finite when supplied");
      const auto regionConfig = config("Regions");
      std::map<std::string, mc_solver::SSMPFLConstraint::RegionParameters> regions;
      for(const auto & name : regionConfig.keys())
      {
        const auto entry = regionConfig(name);
        mc_solver::SSMPFLConstraint::RegionParameters p;
        p.forceMaxN = entry("ForceMaxN");
        p.stiffnessNPerM = entry("StiffnessNPerM");
        p.distanceMarginM = entry("DistanceMarginM", 0.0);
        regions.emplace(name, p);
      }
      return std::make_shared<mc_solver::SSMPFLConstraint>(
          solver.robot(robotIndex), static_cast<unsigned int>(capacity), solver.dt(), std::move(regions), stoppingTime,
          minMobility, minMassEigenvalue, weight);
    });

} // namespace
