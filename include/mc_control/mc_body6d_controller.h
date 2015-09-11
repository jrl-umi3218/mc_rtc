#pragma once

#include <mc_control/mc_controller.h>

#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/CoMTask.h>

namespace mc_control
{

struct MCBody6dController : public MCController
{
public:
  MCBody6dController();

  virtual void reset(const ControllerResetData & reset_data) override;

  /* Specific to 6d controller */
  bool change_ef(const std::string & ef_name);

  bool translate_ef(const Eigen::Vector3d & t);

  bool rotate_ef(const Eigen::Matrix3d & m);
public:
  std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
  std::shared_ptr<mc_tasks::CoMTask> comTask;
};

}
