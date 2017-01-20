#include <mc_solver/ContactConstraint.h>

#include <mc_rtc/logging.h>

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
    throw(std::string("bad constraint type"));
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
