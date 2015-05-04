#ifndef _H_MCSEQCONTROLLER_H_
#define _H_MCSEQCONTROLLER_H_

#include <mc_control/mc_controller.h>

#include <mc_rbdyn/stance.h>
#include <mc_tasks/StabilityTask.h>
#include <Tasks/QPConstr.h>

namespace mc_control
{

struct SeqAction;

struct MCSeqController : public MCController
{
public:
  MCSeqController(const std::string & env_path, const std::string & env_name, const std::string & seq_path);

  virtual bool run() override;

  virtual void reset(const ControllerResetData & reset_data) override;
  /* Services */
public:
  /* Sequence playing logic */
  bool paused;
  unsigned int stanceIndex;
  std::vector<mc_rbdyn::Stance> stances;
  std::vector<mc_rbdyn::StanceConfig> configs;
  std::vector< std::shared_ptr<mc_rbdyn::StanceAction> > actions;
  std::vector< std::shared_ptr<SeqAction> > seq_actions;

  /* Tasks and constraints specific to seq controller */
  mc_solver::RobotEnvCollisionsConstraint collsConstraint;
  std::shared_ptr<tasks::qp::BoundedSpeedConstr> constSpeedConstr;
  std::shared_ptr<mc_tasks::StabilityTask> stabilityTask;
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
