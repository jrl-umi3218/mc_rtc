#ifndef _H_MCBODY6DCONTROLLER_H_
#define _H_MCBODY6DCONTROLLER_H_

#include <mc_rbdyn/robot.h>
#include <mc_control/mc_solver/qpsolver.h>

#include <mc_robots/hrp2_drc.h>
#include <mc_robots/env.h>

#include <Tasks/QPTasks.h>
#include <mc_tasks/EndEffectorTask.h>

#include <mc_control/mc_controller.h>

namespace mc_control
{

struct MCBody6dController : public MCController
{
public:
  MCBody6dController();

  virtual void reset(const std::vector< std::vector<double> > & q) override;

  /* Specific to 6d controller */
  bool change_ef(const std::string & ef_name);

  bool translate_ef(const Eigen::Vector3d & t);

  bool rotate_ef(const Eigen::Matrix3d & m);
public:
  std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
};

}

#endif
