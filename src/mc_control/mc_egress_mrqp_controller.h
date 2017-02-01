#pragma once

#include <mc_rbdyn/Robots.h>
#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/TrajectoryTask.h>
#include <mc_tasks/CoMTask.h>

#include <mc_control/api.h>

namespace mc_control
{

struct EgressMRPhaseExecution;

struct MC_CONTROL_DLLAPI MCEgressMRQPController : MCController
{
  public:
    enum EgressPhase
    {
      START = 1,
      REMOVERIGHTFOOT,
      REMOVELEFTFOOT,
      MOVELEFTFOOT,
      ROTATELAZY,
      MOVECOMLEFT,
      MOVECOMRIGHT,
      CENTERCOM,
      MOVECOMFORCELEFT,
      REPLACELEFTFOOT,
      REPLACERIGHTFOOT,
      STANDUP,
      PUTDOWNRIGHTFOOT,
      OPENGRIPPER,
      REMOVEHAND,
      REPLACEHAND
    };
  public:
    MCEgressMRQPController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt);

    virtual bool run() override;

    virtual void reset(const ControllerResetData & reset_data) override;

    virtual bool play_next_stance() override;

    virtual void addCollision(const mc_rbdyn::Collision& coll);

    virtual std::vector<std::string> supported_robots() const override;
  protected:
    void resetBasePose();
    void resetWheelTransform();
    void resetLazyTransform();

  public:
    std::shared_ptr<mc_tasks::CoMTask> comTask;
    std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
    std::shared_ptr<mc_tasks::TrajectoryTask> trajTask;
    std::shared_ptr<tasks::qp::PostureTask> lazyPostureTask;
    std::shared_ptr<mc_tasks::OrientationTask> torsoOriTask;
    std::vector<mc_rbdyn::Contact> egressContacts;

  private:
    std::shared_ptr<tasks::qp::PostureTask> polarisPostureTask;
    mc_solver::KinematicsConstraint polarisKinematicsConstraint;
    mc_solver::CollisionsConstraint collsConstraint;
    EgressPhase curPhase;
    std::shared_ptr<EgressMRPhaseExecution> execPhase;
};

}

SIMPLE_CONTROLLER_CONSTRUCTOR("EgressMRQP", mc_control::MCEgressMRQPController)
