#include <mc_tasks/PostureTask.h>

#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

PostureTask::PostureTask(const mc_solver::QPSolver & solver, unsigned int rIndex, double stiffness, double weight):
  robot_(solver.robots().robot(rIndex)),
  pt_(solver.robots().mbs(), rIndex,
      robot_.mbc().q,
      stiffness, weight),
  dt_(solver.dt()),
  eval_(pt_.eval()),
  speed_(pt_.eval())
{
  eval_.setZero();
  speed_.setZero();
  name_ = std::string("posture_") + robot_.name();
}

void PostureTask::reset()
{
  pt_.posture(robot_.mbc().q);
}

void PostureTask::selectActiveJoints(mc_solver::QPSolver & solver,
                        const std::vector<std::string> & activeJointsName)
{
  std::vector<std::string> unactiveJoints = {};
  for(const auto & j : robot_.mb().joints())
  {
    if(j.dof() && std::find(activeJointsName.begin(),
                            activeJointsName.end(),
                            j.name()) == activeJointsName.end())
    {
      unactiveJoints.push_back(j.name());
    }
  }
  selectUnactiveJoints(solver, unactiveJoints);
}

void PostureTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & unactiveJointsName)
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
  speed_ = (pt_.eval() - eval_)/dt_;
  eval_ = pt_.eval();
}

void PostureTask::posture(const std::vector<std::vector<double>> & p)
{
  pt_.posture(p);
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

}

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function("posture",
  [](mc_solver::QPSolver & solver,
     const mc_rtc::Configuration & config)
  {
    auto t = std::make_shared<mc_tasks::PostureTask>(solver, config("robotIndex"), config("stiffness"), config("weight"));
    t->load(solver, config);
    if(config.has("posture"))
    {
      t->posture(config("posture"));
    }
    return t;
  });

}
