#pragma once

#include <mc_tasks/EndEffectorTask.h>

namespace mc_tasks
{

/* As EndEffectorTask, RelativeEndEffectorTask aims at controlling an
 * end-effector. The main difference is that the control is done relatively to
 * a body of the robot rather than the world frame */

struct RelativeEndEffectorTask : public EndEffectorTask
{
public:
  RelativeEndEffectorTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex, unsigned int relBodyIdx = 0, double stiffness = 10.0, double weight = 1000.0);

  virtual void resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex) override;

  virtual void add_ef_pose(const sva::PTransformd & dtr) override;

  virtual void set_ef_pose(const sva::PTransformd & tf) override;

  virtual void update() override;

  virtual sva::PTransformd get_ef_pose() override;
private:
  unsigned int relBodyIdx;
};

}
