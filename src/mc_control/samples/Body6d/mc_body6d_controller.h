#pragma once

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCBody6dController : public MCController
{
public:
  MCBody6dController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);

  virtual void reset(const ControllerResetData & reset_data) override;

  virtual bool change_ef(const std::string & ef_name) override;

  virtual bool move_ef(const Eigen::Vector3d & t, const Eigen::Matrix3d & m) override;

  bool translate_ef(const Eigen::Vector3d & t);

  bool rotate_ef(const Eigen::Matrix3d & m);

public:
  std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
  std::shared_ptr<mc_tasks::CoMTask> comTask;
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("Body6d", mc_control::MCBody6dController)
