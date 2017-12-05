#pragma once

#include <mc_tasks/MetaTask.h>
#include <mc_tasks/SurfaceTransformTask.h>

namespace mc_tasks
{

/*! \brief Hybrid position-force control on a contacting end-effector.
 *
 * ...
 */
struct MC_TASKS_DLLAPI AdmittanceTask: MetaTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const Eigen::Vector3d DEFAULT_MAX_TRANS_VEL = Eigen::Vector3d(0.1, 0.1, 0.1);  // [m] / [s]
  const Eigen::Vector3d DEFAULT_MAX_TRANS_POS = Eigen::Vector3d(0.1, 0.1, 0.1);  // [m]
  const Eigen::Vector3d DEFAULT_MAX_RPY_VEL = Eigen::Vector3d(0.1, 0.1, 0.1);  // [rad] / [s]
  const Eigen::Vector3d DEFAULT_MAX_RPY_POS = Eigen::Vector3d(0.5, 0.5, 0.5);  // [rad]

  /*! \brief Constructor ...
   *
   * \param robots Robots where the task will be applied.
   *
   * \param robotIndex Which robot among the robots.
   *
   * \param robotSurface Name of the robot surface frame controlled by the
   * task, in which the desired wrench will also be expressed.
   *
   * \param timestep Timestep of the controller.
   *
   * \param forceAdmittance Vector of positive force admittance coefficients.
   *
   * \param torqueAdmittance Vector of positive torque admittance coefficients.
   *
   * \param stiffness Stiffness of the underlying SurfaceTransform task.
   *
   * \param weight Weight of the underlying SurfaceTransform task.
   *
   * \throws If the body the task is attempting to control does not have a
   * sensor attached to it.
   *
   * Admittance coefficients are set to zero by default, in which case the task
   * behaves like a usual SurfaceTransform task.
   *
   */
  AdmittanceTask(const std::string & robotSurface,
      const mc_rbdyn::Robots & robots,
      unsigned robotIndex,
      double timestep,
      const sva::ForceVecd & admittance = sva::ForceVecd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
      double stiffness = 5.0, double weight = 1000.0);

  /*! \brief Reset the task
   *
   * Set the end effector objective to the current position of the
   * end-effector.
   *
   */
  virtual void reset();

  /*! \brief Get the admittance coefficients of the task
   *
   */
  const sva::ForceVecd & admittance()
  {
    return admittance_;
  }

  /*! \brief Set the admittance coefficients of the task
   *
   * \param admittance Vector of positive admittance coefficients
   *
   */
  void admittance(const sva::ForceVecd & admittance)
  {
    admittance_ = admittance;
  }

  /*! \brief Get target translation and orientation for the position task
   *
   */
  const sva::PTransformd & targetPose()
  {
    return X_0_target_;
  }

  /*! \brief Set target position and orientation
   *
   * \param X_0_target Plucker transform from the world frame to the target frame.
   *
   */
  void targetPose(const sva::PTransformd & X_0_target)
  {
    X_0_target_ = X_0_target;
  }

  /*! \brief Get the target wrench in the surface frame
   *
   */
  const sva::ForceVecd & targetWrench()
  {
    return targetWrench_;
  }

  /*! \brief Set the target wrench in the surface frame
   *
   * \param wrench Target wrench in the surface frame
   *
   */
  void targetWrench(const sva::ForceVecd& wrench)
  {
    targetWrench_ = wrench;
  }

  /*! \brief Set the task stiffness.
   *
   * Damping is automatically set to 2 * sqrt(stiffness).
   *
   * \param stiffness Task stiffness
   *
   */
  void stiffness(double stiffness);

  /*! \brief Returns the task stiffness */
  double stiffness() const;

  /*! \brief Set the task weight
   *
   * \param w Task weight
   *
   */
  virtual void weight(double w);
  
  /*! \brief Returns the task weight */
  virtual double weight() const;

  /*! \brief Set the output-dimension-wise weighting vector of the task 
   * 
   * \param dimW Vector whose dimension is the number of output coordinates of
   * the task (e.g. 3 for a position task)
   *
   */
  virtual void dimWeight(const Eigen::VectorXd & dimW) override;

  /*! \brief Get the output-dimension-wise weighting vector of the task */
  virtual Eigen::VectorXd dimWeight() const override;

  virtual void selectActiveJoints(mc_solver::QPSolver & solver,
                                  const std::vector<std::string> & activeJointsName) override;

  virtual void selectUnactiveJoints(mc_solver::QPSolver & solver,
                                    const std::vector<std::string> & unactiveJointsName) override;

  virtual void resetJointsSelector(mc_solver::QPSolver & solver) override;

  /*! \brief Returns the task's error */
  virtual Eigen::VectorXd eval() const override
  {
    return surfaceTask_->eval();
  }

  /*! \brief Returns the task's speed */
  virtual Eigen::VectorXd speed() const override
  {
    return surfaceTask_->speed();
  }

  /*! \brief Set the maximum translation velocity of the task */
  void maxTransVel(const Eigen::Vector3d & maxTransVel)
  {
    if ((maxTransVel.array() <= 0.).any())
    {
      LOG_ERROR("discarding maxTransVel update as it is not positive");
      return;
    }
    maxTransVel_ = maxTransVel;
  }

  /*! \brief Set the maximum translation of the task */
  void maxTransPos(const Eigen::Vector3d & maxTransPos)
  {
    if ((maxTransPos.array() <= 0.).any())
    {
      LOG_ERROR("discarding maxTransPos update as it is not positive");
      return;
    }
    maxTransPos_ = maxTransPos;
  }

  /*! \brief Set the maximum angular velocity of the task */
  void maxRpyVel(const Eigen::Vector3d & maxRpyVel)
  {
    if ((maxRpyVel.array() <= 0.).any())
    {
      LOG_ERROR("discarding maxRpyVel update as it is not positive");
      return;
    }
    maxRpyVel_ = maxRpyVel;
  }

  /*! \brief Set the maximum angular position of the task */
  void maxRpyPos(const Eigen::Vector3d & maxRpyPos)
  {
    if ((maxRpyPos.array() <= 0.).any())
    {
      LOG_ERROR("discarding maxRpyPos update as it is not positive");
      return;
    }
    maxRpyPos_ = maxRpyPos;
  }

private:
  sva::PTransformd computePose();

  const mc_rbdyn::Surface & surface_;
  std::shared_ptr<SurfaceTransformTask> surfaceTask_;
  sva::ForceVecd wrenchError_;
  sva::PTransformd X_0_target_;
  sva::ForceVecd targetWrench_;
  const mc_rbdyn::Robot & robot_;
  const mc_rbdyn::ForceSensor & sensor_;
  double timestep_;
  double forceThresh_, torqueThresh_;
  std::pair<double,double> forceGain_, torqueGain_;
  sva::ForceVecd admittance_;
  // sva::PTransformd X_target_delta_;
  Eigen::Vector3d trans_target_delta_;
  Eigen::Vector3d rpy_target_delta_;
  Eigen::Vector3d maxTransPos_;
  Eigen::Vector3d maxTransVel_;
  Eigen::Vector3d maxRpyPos_;
  Eigen::Vector3d maxRpyVel_;

  virtual void addToSolver(mc_solver::QPSolver & solver) override;
  virtual void removeFromSolver(mc_solver::QPSolver & solver) override;
  virtual void update() override;
};

}
