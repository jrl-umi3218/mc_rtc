/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/MetaTask.h>

namespace mc_tasks
{

namespace force
{

namespace
{
static const std::pair<double, double> defaultFGain = {0.02, 0.005};
static const std::pair<double, double> defaultTGain = {0.2, 0.05};
} // namespace

/*! \brief Add a contact in a compliant manner
 *
 * Uses an mc_tasks::EndEffectorTask to drive the selected body until certain
 * force and torque thresholds are reached.
 *
 * This is a force-compliant variant of mc_tasks::AddContactTask that should be
 * used when a force sensor is available.
 */
struct MC_TASKS_DLLAPI ComplianceTask : MetaTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! \brief Constructor with dof restrictions
   *
   * \param robots Robots where the task will be applied
   *
   * \param robotIndex Which robot among the robots
   *
   * \param body Name of the body controlled by this task.
   *
   * \param timestep Timestep of the controller
   *
   * \param dof Allows to enable/disable some axis in the control/wrench
   * monitoring
   *
   * \param stiffness Stiffness of the task
   *
   * \param weight Weight of the task
   *
   * \param forceThresh Force threshold to reach
   *
   * \param Torque threshold to reach
   *
   * \param forceGain PD gains on the force part
   *
   * \param torqueGain PD gains on the torque part
   *
   * \throws If the body the task is attempting to control does not have a
   * sensor attached to it
   *
   */
  ComplianceTask(const mc_rbdyn::Robots & robots,
                 unsigned int robotIndex,
                 const std::string & body,
                 double timestep,
                 const Eigen::Matrix6d & dof = Eigen::Matrix6d::Identity(),
                 double stiffness = 5.0,
                 double weight = 1000.0,
                 double forceThresh = 3.,
                 double torqueThresh = 1.,
                 std::pair<double, double> forceGain = defaultFGain,
                 std::pair<double, double> torqueGain = defaultTGain);

  /*! \brief Constructor
   *
   * \param robots Robots where the task will be applied
   *
   * \param robotIndex Which robot among the robots
   *
   * \param body Name of the body controlled by this task.
   *
   * \param timestep Timestep of the controller
   *
   * \param stiffness Stiffness of the task
   *
   * \param weight Weight of the task
   *
   * \param forceThresh Force threshold to reach
   *
   * \param Torque threshold to reach
   *
   * \param forceGain PD gains on the force part
   *
   * \param torqueGain PD gains on the torque part
   *
   * \throws If the body the task is attempting to control does not have a
   * sensor attached to it
   *
   */
  ComplianceTask(const mc_rbdyn::Robots & robots,
                 unsigned int robotIndex,
                 const std::string & body,
                 double timestep,
                 double stiffness = 5.0,
                 double weight = 1000.0,
                 double forceThresh = 3.,
                 double torqueThresh = 1.,
                 std::pair<double, double> forceGain = defaultFGain,
                 std::pair<double, double> torqueGain = defaultTGain);

  /*! \brief Reset the task
   *
   * Set the end effector objective to the current position of the end-effector
   *
   */
  void reset() override;

  /*! \brief Get the filtered wrench used by the task as a measure */
  sva::ForceVecd getFilteredWrench() const;

  /*! \brief Modify the target wrench */
  void setTargetWrench(const sva::ForceVecd & wrench)
  {
    obj_ = wrench;
  }

  /*! \brief Get the current target wrench */
  sva::ForceVecd getTargetWrench()
  {
    return obj_;
  }

  /*! \brief Set the task stiffness */
  void stiffness(double s)
  {
    efTask_->positionTask->stiffness(s);
    efTask_->orientationTask->stiffness(s);
  }
  /*! \brief Get the task stiffness */
  double stiffness()
  {
    return efTask_->positionTask->stiffness();
  }

  /*! \brief Set the task weight */
  void weight(double w)
  {
    efTask_->positionTask->weight(w);
    efTask_->orientationTask->weight(w);
  }
  /*! \brief Get the task weight */
  double weight()
  {
    return efTask_->positionTask->weight();
  }

  /*! \brief Set the force threshold */
  void forceThresh(double t)
  {
    forceThresh_ = t;
  }
  /*! \brief Get the force threshold */
  double forceThresh()
  {
    return forceThresh_;
  }

  /*! \brief Set the torque threshold */
  void torqueThresh(double t)
  {
    torqueThresh_ = t;
  }
  /*! \brief Get the torque threshold */
  double torqueThresh()
  {
    return torqueThresh_;
  }

  /*! \brief Set the force gain */
  void forceGain(std::pair<double, double> t)
  {
    forceGain_ = t;
  }
  /*! \brief Get the force gain */
  std::pair<double, double> forceGain()
  {
    return forceGain_;
  }

  /*! \brief Set the torque gain */
  void torqueGain(std::pair<double, double> t)
  {
    torqueGain_ = t;
  }
  /*! \brief Get the torque gain */
  std::pair<double, double> torqueGain()
  {
    return torqueGain_;
  }

  /*! \brief Set the current dof matrix */
  void dof(const Eigen::Matrix6d & dof)
  {
    dof_ = dof;
  }
  /*! \brief Get the current dof matrix */
  Eigen::Matrix6d dof()
  {
    return dof_;
  }

  void dimWeight(const Eigen::VectorXd & dimW) override;

  Eigen::VectorXd dimWeight() const override;

  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & activeJointsName,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

  void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  Eigen::VectorXd eval() const override
  {
    return wrench_.vector();
  }

  Eigen::VectorXd speed() const override
  {
    return robots_.robot(rIndex_).mbc().bodyVelW[robots_.robot(rIndex_).bodyIndexByName(sensor_.parentBody())].vector();
  }

private:
  sva::PTransformd computePose();

  std::shared_ptr<EndEffectorTask> efTask_;
  sva::ForceVecd wrench_;
  sva::ForceVecd obj_;
  sva::ForceVecd error_;
  sva::ForceVecd errorD_;
  const mc_rbdyn::Robots & robots_;
  unsigned int rIndex_;
  const mc_rbdyn::ForceSensor & sensor_;
  double timestep_;
  double forceThresh_, torqueThresh_;
  std::pair<double, double> forceGain_, torqueGain_;
  Eigen::Matrix6d dof_;
  std::function<double(double)> clampTrans_, clampRot_;

  void addToSolver(mc_solver::QPSolver & solver) override;

  void removeFromSolver(mc_solver::QPSolver & solver) override;

  void update(mc_solver::QPSolver &) override;
};

} // namespace force

} // namespace mc_tasks
