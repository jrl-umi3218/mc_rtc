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

struct MC_TASKS_DLLAPI ComplianceTask: MetaTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

  virtual void resetTask(const mc_rbdyn::Robots& robots, unsigned int robotIndex);

  const sva::ForceVecd & getFilteredWrench() const;

  void setTargetWrench(const sva::ForceVecd& wrench)
  {
    obj_ = wrench;
  }

  const Eigen::Vector6d eval()
  {
    return wrench_.vector();
  }

  const Eigen::Vector6d speed()
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
