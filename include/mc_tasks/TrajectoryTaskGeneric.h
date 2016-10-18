#pragma once

#include <mc_rbdyn/robot.h>
#include <mc_solver/QPSolver.h>
#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>
#include <mc_tasks/MetaTask.h>

namespace mc_tasks
{

/*! \brief Generic wrapper for a tasks::qp::TrajectoryTask
 *
 * This task is meant to be derived to build actual tasks
 *
 */
template<typename T>
struct TrajectoryTaskGeneric : public MetaTask
{

  /*! \brief Constructor (auto damping)
   *
   * This is a simple constructor alternative. Damping is set to
   * 2*sqrt(stiffness). This is the most appropriate constructor to use
   * TrajectoryTask as a SetPointTask
   *
   * \param robots Robots used in the task
   *
   * \param robotIndex Index of the robot controlled by the task
   *
   * \param stiffness Stiffness of the task
   *
   * \param weight Weight of the task
   *
   */
  TrajectoryTaskGeneric(const mc_rbdyn::Robots & robots, unsigned int robotIndex,
                        double stiffness, double weight);

  virtual ~TrajectoryTaskGeneric();

  virtual void removeFromSolver(mc_solver::QPSolver & solver) override;

  virtual void addToSolver(mc_solver::QPSolver & solver) override;

  virtual void update() override;

  /*! \brief Reset the task objective to the current robot's state */
  virtual void reset() = 0;

  /*! \brief Set the trajectory reference velocity
   *
   * \param vel New reference velocity
   *
   */
  void refVel(const Eigen::VectorXd & vel);

  /*! \brief Set the trajectory reference acceleration
   *
   * \param accel New reference acceleration
   *
   */
  void refAccel(const Eigen::VectorXd & accel);

  /*! \brief Set the task stiffness/damping
   *
   * Damping is automatically set to 2*sqrt(stiffness)
   *
   * \param stiffness Task stiffness
   *
   */
  void stiffness(double stiffness);

  /*! \brief Set both stiffness and damping
   *
   * \param stiffness Task stiffness
   *
   * \param damping Task damping
   *
   */
  void setGains(double stifness, double damping);

  /*! \brief Get the current task stiffness */
  double stiffness() const;

  /*! \brief Get the current task damping */
  double damping() const;

  /*! \brief Set the task weight
   *
   * \param w Task weight
   *
   */
  void weight(double w);

  /*! \brief Returns the task weight */
  double weight() const;

  /*! \brief Set the task's dimension weight vector
   *
   * It is the caller responsibility to ensure the dimensionality fits the
   * underlying tasks' error function
   *
   * \param dimW The new task's dimension weight vector
   *
   */
  void dimWeight(const Eigen::VectorXd & dimW);

  /*! \brief Get the current task's dim weight vector */
  Eigen::VectorXd dimWeight() const;

  /*! \brief Returns the current task error */
  virtual Eigen::VectorXd eval() const override;

  /*! \brief Returns the current task speed */
  virtual Eigen::VectorXd speed() const override;
protected:
  /*! This function should be called to finalize the task creation, it will
   * create the actual tasks objects */
  template<typename ... Args>
  void finalize(Args && ... args);
protected:
  const mc_rbdyn::Robots & robots;
  unsigned int rIndex;
  std::shared_ptr<T> errorT = nullptr;
private:
  double stiff;
  double damp;
  double wt;
  bool inSolver = false;
  std::shared_ptr<tasks::qp::TrajectoryTask> trajectoryT = nullptr;
};

}

#include <mc_tasks/TrajectoryTaskGeneric.hpp>
