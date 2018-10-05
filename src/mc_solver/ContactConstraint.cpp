#include <mc_rtc/logging.h>
#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/ContactConstraint.h>

namespace mc_solver
{

ContactConstraint::ContactConstraint(double timeStep, ContactType contactType, bool dynamics)
{
  if(contactType == Acceleration)
  {
    contactConstr.reset(new tasks::qp::ContactAccConstr());
  }
  else if(contactType == Velocity)
  {
    contactConstr.reset(new tasks::qp::ContactSpeedConstr(timeStep));
  }
  else if(contactType == Position)
  {
    contactConstr.reset(new tasks::qp::ContactPosConstr(timeStep));
  }
  else
  {
    LOG_ERROR("Trying to create a contact constraint from an unknown contact constraint type")
    LOG_ERROR_AND_THROW(std::runtime_error, std::string("bad constraint type"))
  }
  if(dynamics)
  {
    posLambdaConstr.reset(new tasks::qp::PositiveLambda());
  }
}

void ContactConstraint::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver)
{
  if(contactConstr)
  {
    contactConstr->addToSolver(mbs, solver);
  }
  if(posLambdaConstr)
  {
    posLambdaConstr->addToSolver(mbs, solver);
  }
}

void ContactConstraint::removeFromSolver(tasks::qp::QPSolver & solver)
{
  if(contactConstr)
  {
    contactConstr->removeFromSolver(solver);
  }
  if(posLambdaConstr)
  {
    posLambdaConstr->removeFromSolver(solver);
  }
}

} // namespace mc_solver

namespace
{

static bool registered = mc_solver::ConstraintSetLoader::register_load_function(
    "contact",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      std::string cTypeStr = config("contactType", std::string{"velocity"});
      auto cType = mc_solver::ContactConstraint::Velocity;
      if(cTypeStr == "acceleration")
      {
        cType = mc_solver::ContactConstraint::Acceleration;
      }
      else if(cTypeStr == "position")
      {
        cType = mc_solver::ContactConstraint::Position;
      }
      else if(cTypeStr != "velocity")
      {
        LOG_ERROR("Stored contact type for contact constraint not recognized, default to velocity")
        LOG_WARNING("(Read: " << cTypeStr << ", expect one of: acceleration, velocity, position)")
      }
      bool dynamics = config("dynamics", true);
      return std::make_shared<mc_solver::ContactConstraint>(solver.dt(), cType, dynamics);
    });
}
