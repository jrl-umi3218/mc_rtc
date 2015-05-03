#ifndef _H_MCSEQCONTROLLER_H_
#define _H_MCSEQCONTROLLER_H_

#include <mc_control/mc_controller.h>

#include <mc_rbdyn/stance.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/CoMTask.h>

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
  std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
  std::shared_ptr<mc_tasks::CoMTask> comTask;

  bool paused;
  unsigned int stanceIndex;
  std::vector<mc_rbdyn::Stance> stances;
  std::vector< std::shared_ptr<mc_rbdyn::StanceAction> > actions;
  std::vector< std::shared_ptr<SeqAction> > seq_actions;
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
