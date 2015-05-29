#ifndef _H_MCEGRESSMRQPCONTROLLER_H_
#define _H_MCEGRESSMRQPCONTROLLER_H_

#include <mc_rbdyn/robot.h>
#include <mc_control/mc_solver/qpsolver.h>
#include <mc_control/mc_mrqp_controller.h>
#include <mc_tasks/EndEffectorTask.h>

namespace mc_control
{

struct MCEgressMRQPController : MCMRQPController
{
  public:
    MCEgressMRQPController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule> >& env_modules);

    virtual bool run() override;

    virtual void reset(const ControllerResetData & reset_data) override;
  protected:
    void resetBasePose();
    void resetWheelTransform();
  private:
    std::shared_ptr<tasks::qp::PostureTask> polarisPostureTask;
    std::shared_ptr<tasks::qp::PostureTask> lazyPostureTask;
    mc_solver::KinematicsConstraint polarisKinematicsConstraint;
    std::vector<mc_rbdyn::MRContact> egressContacts;
    mc_solver::CollisionsConstraint collsConstraint;
};

}
#endif
