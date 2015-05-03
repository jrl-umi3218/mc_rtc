#include <mc_control/mc_seq_controller.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_control
{

MCSeqController::MCSeqController(const std::string & env_path, const std::string & env_name, const std::string & seq_path)
: MCController(env_path, env_name), paused(false), stanceIndex(0), seq_actions(0)
{
  /* Load plan */
  loadStances(seq_path, stances, actions);
  assert(stances.size() == actions.size());
  for(size_t i = 0; i < stances.size(); ++i)
  {
    seq_actions.push_back(seqActionFromStanceAction(stances[i], *(actions[i].get())));
  }

  /* Setup the QP */
  sva::PTransformd leftFootSurfTf = robot().surfaces["LFullSole"]->X_0_s(robot());
  auto q = robot().mbc->q;
  q[0] = {1, 0, 0, 0, 0, 0, -leftFootSurfTf.translation().z()};
  robot().mbc->q = q;
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));

  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(dynamicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->solver.addTask(postureTask.get());
  qpsolver->setContacts({
    mc_rbdyn::Contact(robot().surfaces.at("LFullSole"), env().surfaces.at("Ground")),
    mc_rbdyn::Contact(robot().surfaces.at("RFullSole"), env().surfaces.at("Ground"))
  });
  comTask.reset(new mc_tasks::CoMTask(qpsolver->robots, qpsolver->robots.robotIndex));
  comTask->addToSolver(*qpsolver);
  /* No EF task in the controller initially but we still set-it up for reliable resets */
  efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6", qpsolver->robots, qpsolver->robots.robotIndex));
}

bool MCSeqController::run()
{
  if(!paused && stanceIndex < seq_actions.size())
  {
    if(seq_actions[stanceIndex]->execute(*this))
    {
      /*FIXME Should pause here to mimic ask_step */
      stanceIndex++;
    }
  }
  return MCController::run();
}

void MCSeqController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  if(reset_data.contacts.size())
  {
    qpsolver->setContacts(reset_data.contacts);
  }
  else
  {
    qpsolver->setContacts({
      mc_rbdyn::Contact(robot().surfaces.at("LFullSole"), env().surfaces.at("Ground")),
      mc_rbdyn::Contact(robot().surfaces.at("RFullSole"), env().surfaces.at("Ground"))
    });
  }
  comTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex);
  efTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex);
}

std::shared_ptr<SeqAction> seqActionFromStanceAction(mc_rbdyn::Stance & stance, mc_rbdyn::StanceAction & action)
{
  /*FIXME Implement stuff*/
  auto res = std::shared_ptr<SeqAction>(new SeqAction());
  res->steps.push_back(SeqStep());
  return res;
}

SeqAction::SeqAction()
: currentStep(0), steps(0)
{
}

bool SeqAction::execute(MCSeqController & controller)
{
  if(steps[currentStep].eval(controller))
  {
    currentStep++;
    if(currentStep == steps.size())
    {
      return true;
    }
  }
  return false;
}

bool SeqStep::eval(MCSeqController & controller)
{
  return false;
}

}
