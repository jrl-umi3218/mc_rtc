#ifndef _H_MCCOMCONTROLLER_H_
#define _H_MCCOMCONTROLLER_H_

#include <mc_control/mc_controller.h>

#include <mc_tasks/CoMTask.h>

namespace mc_control
{

struct MCCoMController : public MCController
{
public:
  MCCoMController();

  virtual void reset(const ControllerResetData & reset_data) override;

  /* Services */
  bool move_com(const Eigen::Vector3d & v);
public:
  std::shared_ptr<mc_tasks::CoMTask> comTask;
};

}

#endif
