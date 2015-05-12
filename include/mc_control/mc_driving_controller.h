#ifndef _H_MCDRIVINGCONTROLLER_H_
#define _H_MCDRIVINGCONTROLLER_H_

#include <mc_rbdyn/robot.h>
#include <mc_control/mc_solver/qpsolver.h>
#include <mc_control/mc_mrqp_controller.h>
#include <mc_tasks/EndEffectorTask.h>

namespace mc_control
{

struct MCDrivingController : MCMRQPController
{
  public:
    MCDrivingController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule> >& env_modules);

    bool changeWheelAngle(double theta);

    bool changeAnkleAngle(double theta);

    bool run() override;

  private:
    std::shared_ptr<mc_rbdyn::RobotModule> polarisModule;
    std::vector<std::shared_ptr<mc_rbdyn::RobotModule> > robotModules;
    mc_tasks::EndEffectorTask ef_task;
    mc_solver::KinematicsConstraint polarisKinematicsConstraint;
};

}
#endif
