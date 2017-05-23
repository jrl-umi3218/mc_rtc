#pragma once

#include <mc_control/mc_controller.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/EndEffectorTask.h>

#include <mc_control/api.h>

namespace mc_control
{

struct EgressPhaseExecution;

struct MC_CONTROL_DLLAPI MCEgressController : public MCController
{
public:
  enum EgressPhase
  {
    START = 1,
    MOVEFOOTINSIDE,
    REMOVEHAND,
    ROTATEBODY,
    MOVEFOOTOUT,
    CORRECTLFOOT,
    CORRECTRFOOT,
    CORRECTBODY,
    STANDUP
  };
public:
  MCEgressController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);

  virtual void reset(const ControllerResetData & reset_data) override;

  virtual bool run() override;
  /* Services */
  virtual bool change_ef(const std::string & ef_name) override;

  virtual bool move_ef(const Eigen::Vector3d & v, const Eigen::Matrix3d & m) override;

  virtual bool move_com(const Eigen::Vector3d & v) override;

  virtual bool play_next_stance() override;

  virtual std::vector<std::string> supported_robots() const override;
public:
  mc_solver::CollisionsConstraint collsConstraint;
  std::shared_ptr<mc_tasks::CoMTask> comTask;
  std::shared_ptr<mc_tasks::OrientationTask> oriTask;
  std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
private:
  void resetBasePose();
  EgressPhase phase;
  std::shared_ptr<EgressPhaseExecution> phaseExec;
};

}

SIMPLE_CONTROLLER_CONSTRUCTOR("Egress", mc_control::MCEgressController)
