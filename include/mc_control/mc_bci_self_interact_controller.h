#pragma once

#include <mc_control/mc_controller.h>

#include <mc_tasks/RelativeEndEffectorTask.h>
#include <mc_tasks/CoMTask.h>

namespace tf2_ros
{
  class TransformBroadcaster;
}

namespace mc_control
{

struct MCBCISelfInteractController : public MCController
{
public:
  MCBCISelfInteractController();

  virtual bool run() override;

  virtual void reset(const ControllerResetData & reset_data) override;

  virtual bool read_msg(std::string & msg) override;
private:
  std::shared_ptr<mc_tasks::RelativeEndEffectorTask> efTask;
  std::shared_ptr<mc_tasks::CoMTask> comTask;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_caster;
  unsigned int seq;
};

}
