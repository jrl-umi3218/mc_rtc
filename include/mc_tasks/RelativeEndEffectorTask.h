#pragma once

#include <mc_tasks/EndEffectorTask.h>

#include <mc_tasks/api.h>

namespace mc_tasks
{

/*! \brief Controls an end-effector relatively to another body
 *
 * This task is the relative counter-part to mc_tasks::EndEffectorTask.
 * The difference is that the control is done relatively to a body of
 * the robot rather than the world frame.
 *
 */
struct MC_TASKS_DLLAPI RelativeEndEffectorTask : public EndEffectorTask
{
public:
  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param relBodyIdx Index of the body relatively to which the end-effector
   * is controlled. Default to 0 (the robot's base)
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  RelativeEndEffectorTask(const std::string & bodyName,
                          const mc_rbdyn::Robots & robots,
                          unsigned int robotIndex,
                          unsigned int relBodyIdx = 0,
                          double stiffness = 10.0, double weight = 1000.0);
  /*! \brief Constructor with bodyPoint
   *
   * \param bodyPoint Point to be controlled in body coordinates
   *
   * @see RelativeEndEffectorTask
   *
   */
  RelativeEndEffectorTask(const std::string & bodyName,
                          const Eigen::Vector3d & bodyPoint,
                          const mc_rbdyn::Robots & robots,
                          unsigned int robotIndex,
                          unsigned int relBodyIdx = 0,
                          double stiffness = 10.0, double weight = 1000.0);

  virtual void reset() override;

  virtual void add_ef_pose(const sva::PTransformd & dtr) override;

  virtual void set_ef_pose(const sva::PTransformd & tf) override;

  virtual sva::PTransformd get_ef_pose() override;
private:
  unsigned int relBodyIdx;

  virtual void update() override;
};

}
