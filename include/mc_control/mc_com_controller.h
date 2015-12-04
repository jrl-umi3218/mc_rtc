#ifndef _H_MCCOMCONTROLLER_H_
#define _H_MCCOMCONTROLLER_H_

#include <mc_control/mc_controller.h>

#include <mc_tasks/CoMTask.h>

#include <mc_control/api.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCCoMController : public MCController
{
public:
  MCCoMController(double dt);

  virtual void reset(const ControllerResetData & reset_data) override;

  /* Services */
  bool move_com(const Eigen::Vector3d & v);
public:
  std::shared_ptr<mc_tasks::CoMTask> comTask;
};

}

#endif
