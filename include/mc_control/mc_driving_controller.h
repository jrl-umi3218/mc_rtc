#ifndef _H_MCDRIVINGCONTROLLER_H_
#define _H_MCDRIVINGCONTROLLER_H_

#include <mc_rbdyn/robot.h>
#include <mc_control/mc_mrqp_controller.h>
#include <mc_tasks/EndEffectorTask.h>

#include <fstream>

#include <mc_control/api.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCDrivingController : MCMRQPController
{
  public:
    MCDrivingController(double dt, const std::vector<std::shared_ptr<mc_rbdyn::RobotModule> >& env_modules);

    bool changeWheelAngle(double theta);

    bool changeAnkleAngle(double theta);

    bool changeGaze(double pan, double tilt);

    bool changeWristAngle(double yaw);

    bool driving_service(double w, double a, double p, double t);

    virtual bool run() override;

    virtual void reset(const ControllerResetData & reset_data) override;

    virtual bool read_msg(std::string & msg) override;
  protected:
    void resetBasePose();
    void resetWheelTransform();

  private:
    sva::PTransformd graspOffset;
    std::shared_ptr<mc_rbdyn::RobotModule> polarisModule;
    std::vector<std::shared_ptr<mc_rbdyn::RobotModule> > robotModules;
    mc_tasks::EndEffectorTask ef_task;
    std::shared_ptr<tasks::qp::PostureTask> polarisPostureTask;
    mc_solver::KinematicsConstraint polarisKinematicsConstraint;
    std::vector<mc_rbdyn::Contact> drivingContacts;
    mc_solver::CollisionsConstraint collsConstraint;
    unsigned iter_;
    double theta_;
    bool logging_;
    std::ofstream log_ankle_;
    std::ofstream log_wheel_;
    std::ofstream log_ef_;
    std::ofstream log_acc_;
    std::ofstream log_rpy_;
    double tMax_;
    double tMin_;
    mc_tasks::EndEffectorTask head_task;
    mc_tasks::EndEffectorTask lhand_task;

    void lock_head();
    void unlock_head();
    void lock_lhand();
    void unlock_lhand();
    void start_logging();
    void stop_logging();
};

}
#endif
