#ifndef _H_MCEGRESSMRQPCONTROLLER_H_
#define _H_MCEGRESSMRQPCONTROLLER_H_

#include <mc_rbdyn/robot.h>
#include <mc_control/mc_solver/qpsolver.h>
#include <mc_control/mc_mrqp_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/TrajectoryTask.h>
#include <mc_tasks/CoMTask.h>

namespace mc_control
{

struct EgressMRPhaseExecution;

struct MCEgressMRQPController : MCMRQPController
{
  public:
    enum EgressPhase
    {
      START = 1,
      ROTATELAZY,
      MOVECOMLEFT,
      MOVECOMRIGHT,
      MOVECOMFORCELEFT,
      REPLACELEFTFOOT,
      REPLACERIGHTFOOT,
      STANDUP,
      REMOVEHAND
    };
  public:
    MCEgressMRQPController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule> >& env_modules);

    virtual bool run() override;

    virtual void reset(const ControllerResetData & reset_data) override;
  protected:
    void resetBasePose();
    void resetWheelTransform();
    void resetLazyTransform();
    void nextPhase();

  public:
    std::shared_ptr<mc_tasks::CoMTask> comTask;
    std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
    std::shared_ptr<mc_tasks::TrajectoryTask> trajTask;
    std::shared_ptr<tasks::qp::PostureTask> lazyPostureTask;
    std::vector<mc_rbdyn::MRContact> egressContacts;

  private:
    std::shared_ptr<tasks::qp::PostureTask> polarisPostureTask;
    mc_solver::KinematicsConstraint polarisKinematicsConstraint;
    mc_solver::CollisionsConstraint collsConstraint;
    EgressPhase curPhase;
    std::shared_ptr<EgressMRPhaseExecution> execPhase;
};

}
#endif
