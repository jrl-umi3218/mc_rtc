#ifndef _H_MCSEQCONTROLLER_H_
#define _H_MCSEQCONTROLLER_H_

#include <mc_control/mc_controller.h>

#include <mc_rbdyn/stance.h>
#include <mc_tasks/StabilityTask.h>
#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/MoveContactTask.h>
#include <Tasks/QPConstr.h>

namespace mc_control
{

struct ActiGripper
{
public:
  ActiGripper();

  ActiGripper(unsigned int wrenchIndex, double actiForce, double stopForce,
              tasks::qp::ContactId contactId, sva::PTransformd & X_0_s, double maxDist,
              const std::shared_ptr<tasks::qp::PositionTask> & positionTask,
              const std::shared_ptr<tasks::qp::SetPointTask> & positionTaskSp);
public:
  unsigned int wrenchIndex;
  double actiForce;
  double stopForce;
  tasks::qp::ContactId contactId;
  sva::PTransformd X_0_s;
  double maxDist;
  std::shared_ptr<tasks::qp::PositionTask> positionTask;
  std::shared_ptr<tasks::qp::SetPointTask> positionTaskSp;
  bool activated;
  Eigen::Vector3d zVec;
  bool toRemove;
  double targetError;
};

struct SeqAction;

std::vector<mc_solver::Collision> confToColl(const std::vector<mc_rbdyn::StanceConfig::BodiesCollisionConf> & conf);

struct MCSeqController : public MCController
{
public:
  MCSeqController(const std::string & env_path, const std::string & env_name, const std::string & seq_path);

  virtual bool run() override;

  virtual void reset(const ControllerResetData & reset_data) override;
  /* Utils functions called by SeqStep/SeqAction */
  void updateRobotEnvCollisions(const mc_rbdyn::Stance & stance, const mc_rbdyn::StanceConfig & conf);

  void updateSelfCollisions(const mc_rbdyn::Stance & stance, const mc_rbdyn::StanceConfig & conf);

  void updateContacts(const std::vector<mc_rbdyn::Contact> & contacts); /*FIXME Actually update sensorContacts... */

  void updateSolverEqInEq();

  void pre_live();

  void post_live();

  const mc_rbdyn::StanceConfig & curConf();

  const mc_rbdyn::Stance & curStance();

  const mc_rbdyn::StanceAction & curAction();

  const mc_rbdyn::Stance & targetStance();

  const mc_rbdyn::StanceAction & targetAction();

  std::vector<std::string> bodiesFromContacts(const mc_rbdyn::Robot & robot, const std::vector<mc_rbdyn::Contact> & robotContacts);
  /* Services */
public:
  /* Sequence playing logic */
  bool paused;
  bool halted;
  unsigned int stanceIndex;
  std::vector<mc_rbdyn::Stance> stances;
  std::vector<mc_rbdyn::StanceConfig> configs;
  std::vector< std::shared_ptr<mc_rbdyn::StanceAction> > actions;
  std::vector< std::shared_ptr<SeqAction> > seq_actions;
  std::vector<mc_tasks::MetaTask*> metaTasks;
  std::map<std::string, ActiGripper> actiGrippers;

  /* Tasks and constraints specific to seq controller */
  mc_solver::RobotEnvCollisionsConstraint collsConstraint;
  std::shared_ptr<tasks::qp::BoundedSpeedConstr> constSpeedConstr;
  std::shared_ptr<mc_tasks::StabilityTask> stabilityTask;
  std::shared_ptr<mc_tasks::MoveContactTask> moveContactTask;
  std::shared_ptr<mc_tasks::AddRemoveContactTask> addContactTask;
  std::vector< std::shared_ptr<tasks::qp::GripperTorqueTask> > gripperTorqueTasks;
};

std::shared_ptr<SeqAction> seqActionFromStanceAction(mc_rbdyn::Stance & stance, mc_rbdyn::StanceAction & action);

struct SeqStep;

struct SeqAction
{
public:
  SeqAction();

  virtual bool execute(MCSeqController & controller);
public:
  unsigned int currentStep;
  std::vector< SeqStep > steps;
};

struct SeqStep
{
public:
  virtual bool eval(MCSeqController & controller);
};

}

#endif
