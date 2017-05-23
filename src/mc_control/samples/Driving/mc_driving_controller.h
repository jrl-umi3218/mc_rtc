#pragma once

#include <mc_rbdyn/Robots.h>
#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>

#include <fstream>
#include <array>

#include <mc_control/api.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCDrivingController : MCController
{
  public:
    MCDrivingController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);

    bool changeWheelAngle(double theta);

    bool changeAnkleAngle(double theta);

    bool changeGaze(double pan, double tilt);

    bool changeWristAngle(double yaw);

    virtual bool driving_service(double w, double a, double p, double t) override;

    virtual bool run() override;

    virtual void reset(const ControllerResetData & reset_data) override;

    virtual bool read_msg(std::string & msg) override;

    virtual std::vector<std::string> supported_robots() const override;
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

SIMPLE_CONTROLLER_CONSTRUCTOR("Driving", mc_control::MCDrivingController)
