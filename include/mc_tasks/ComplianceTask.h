#ifndef _H_COMPLIANCETASK_H_
#define _H_COMPLIANCETASK_H_

#include <mc_tasks/MetaTask.h>
#include <mc_tasks/EndEffectorTask.h>

#include <mc_rbdyn/calibrator.h>

namespace mc_tasks
{

namespace
{
  static const std::pair<double, double> defaultFGain = {0.02, 0.005};
  static const std::pair<double, double> defaultTGain = {0.2, 0.05};
}

/*! \brief Add a contact in a compliant manner
 *
 * Uses an mc_tasks::EndEffectorTask to drive the selected body until certain
 * force and torque thresholds are reached.
 *
 * This is a force-compliant variant of mc_tasks::AddContactTask that should be
 * used when a force sensor is available.
 */
struct MC_TASKS_DLLAPI ComplianceTask: MetaTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! \brief Constructor with dof restrictions
   *
   * \param robots Robots where the task will be applied
   *
   * \param robotIndex Which robot among the robots
   *
   * \param forceSensor Which force sensor to read data from, the end-effector
   * controlled by this task is the parent body of the force sensor
   *
   * \param ctlWrenches Map holding the force sensor data
   *
   * \param calibrator Calibrator structure used to filter the wrench at the
   * force sensor
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
   */
  ComplianceTask(const mc_rbdyn::Robots & robots,
        unsigned int robotIndex,
        const mc_rbdyn::ForceSensor& forceSensor,
        const std::map<std::string, sva::ForceVecd>& ctlWrenches,
        const mc_rbdyn::ForceSensorsCalibrator& calibrator,
        double timestep,
        const Eigen::Matrix6d& dof = Eigen::Matrix6d::Identity(),
        double stiffness = 5.0, double weight = 1000.0,
        double forceThresh = 3., double torqueThresh = 1.,
        std::pair<double,double> forceGain = defaultFGain,
        std::pair<double,double> torqueGain = defaultTGain);

  /*! \brief Constructor
   *
   * \param robots Robots where the task will be applied
   *
   * \param robotIndex Which robot among the robots
   *
   * \param forceSensor Which force sensor to read data from, the end-effector
   * controlled by this task is the parent body of the force sensor
   *
   * \param ctlWrenches Map holding the force sensor data
   *
   * \param calibrator Calibrator structure used to filter the wrench at the
   * force sensor
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
   */
  ComplianceTask(const mc_rbdyn::Robots & robots,
      unsigned int robotIndex,
      const mc_rbdyn::ForceSensor& forceSensor,
      const std::map<std::string, sva::ForceVecd>& ctlWrenches,
      const mc_rbdyn::ForceSensorsCalibrator& calibrator,
      double timestep,
      double stiffness = 5.0, double weight = 1000.0,
      double forceThresh = 3., double torqueThresh = 1.,
      std::pair<double,double> forceGain = defaultFGain,
      std::pair<double,double> torqueGain = defaultTGain);

  virtual void addToSolver(mc_solver::QPSolver & solver);

  virtual void removeFromSolver(mc_solver::QPSolver & solver);

  virtual void update();

  /*! \brief Reset the task
   *
   * Set the end effector objective to the current position of the end-effector
   *
   */
  virtual void reset();

  /*! \brief Get the filtered wrench used by the task as a measure */
  sva::ForceVecd getFilteredWrench() const;

  /*! \brief Modify the target wrench */
  void setTargetWrench(const sva::ForceVecd& wrench)
  {
    obj_ = wrench;
  }

  /*! \brief Returns the task's error */
  virtual Eigen::VectorXd eval() const override
  {
    return wrench_.vector();
  }

  /*! \brief Returns the task's speed */
  virtual Eigen::VectorXd speed() const override
  {
    return robot_.mbc().bodyVelW[robot_.bodyIndexByName(sensor_.parentBodyName)].vector();
  }

private:
  sva::PTransformd computePose();

  std::shared_ptr<EndEffectorTask> efTask_;
  const std::map<std::string, sva::ForceVecd>& ctlWrenches_;
  sva::ForceVecd wrench_;
  sva::ForceVecd obj_;
  sva::ForceVecd error_;
  sva::ForceVecd errorD_;
  const mc_rbdyn::ForceSensorsCalibrator& calibrator_;
  const mc_rbdyn::Robot& robot_;
  const mc_rbdyn::ForceSensor& sensor_;
  double timestep_;
  double forceThresh_, torqueThresh_;
  std::pair<double,double> forceGain_, torqueGain_;
  Eigen::Matrix6d dof_;
  std::function<double(double)> clampTrans_, clampRot_;
};

}

#endif
