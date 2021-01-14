/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifndef _H_ADDREMOVECONTACTTASK_H_
#define _H_ADDREMOVECONTACTTASK_H_

#include <mc_rbdyn/Robots.h>
#include <mc_solver/BoundedSpeedConstr.h>
#include <mc_tasks/MetaTask.h>
#include <mc_tasks/api.h>

#include <Tasks/QPTasks.h>

namespace mc_rbdyn
{
struct Contact;
}

namespace mc_tasks
{

/*! \brief Add or remove a contact
 *
 * The goal of this task is to move a robot's surface towards a contact
 * or away from a contact depending on construction parameters.
 *
 * The robot's surface will move along its normal axis. It is the
 * programmer responsibility to stop the task when the desired
 * destination has been reached.
 */
struct MC_TASKS_DLLAPI AddRemoveContactTask : public MetaTask
{
public:
  /*! \brief General constructor
   *
   * Constructs an AddRemoveContactTask using information passed by the
   * programmer
   *
   * \param robots Robots involved in the task
   *
   * \param constSpeedConstr Used by the solver to constraint the
   * surface movement
   *
   * \param contact Contact that will be added/removed
   *
   * \param direction Direction of displacement
   *
   * \param speed Speed of the displacement
   *
   * \param stiffness Stiffness of the task
   *
   * \param weight Weight of the task
   *
   * \param userT_0_s If provided, overrides the chosen normal
   * direction
   */
  AddRemoveContactTask(mc_rbdyn::Robots & robots,
                       std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                       mc_rbdyn::Contact & contact,
                       double direction,
                       double speed = 0.01,
                       double stiffness = 2,
                       double weight = 1000,
                       Eigen::Vector3d * userT_0_s = nullptr);

  /*! \brief General constructor with self-managed bounded speed constraint
   *
   * Constructs an AddRemoveContactTask using information passed by the programmer. No mc_solver::BoundedSpeedConstr has
   * to be passed. Instead the constraint is created and managed by the task.
   *
   * \param solver Solver where the task will be used
   *
   * \param contact Contact that will be added/removed
   *
   * \param direction Direction of displacement
   *
   * \param speed Speed of the displacement
   *
   * \param stiffness Stiffness of the task
   *
   * \param weight Weight of the task
   *
   * \param userT_0_s if provided, overrides the chosen normal direction
   */
  AddRemoveContactTask(mc_solver::QPSolver & solver,
                       mc_rbdyn::Contact & contact,
                       double direction,
                       double speed = 0.01,
                       double stiffness = 2,
                       double weight = 1000,
                       Eigen::Vector3d * userT_0_s = nullptr);

  /*! \brief Set the displacement direction
   *
   * \param direction Direction of the displacement
   *
   */
  void direction(double direction);

  /*! \brief Get the desired dislacement speed */
  double speed()
  {
    return speed_;
  }
  /*! \brief Set the desired dislacement speed */
  void speed(double s);

  /*! \brief Get the task stiffness */
  double stiffness()
  {
    return stiffness_;
  }
  /*! \brief Get the task weight */
  double weight()
  {
    return weight_;
  }

  /*! \brief Get the velocity error
   *
   * \returns Velocity error of the LinVelocity task */
  Eigen::Vector3d velError();

  void dimWeight(const Eigen::VectorXd & dimW) override;

  Eigen::VectorXd dimWeight() const override;

  Eigen::VectorXd eval() const override;

  Eigen::VectorXd speed() const override;

public:
  mc_rbdyn::Robots & robots;
  mc_rbdyn::Robot & robot;
  mc_rbdyn::Robot & env;
  std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr;

  std::shared_ptr<mc_rbdyn::Surface> robotSurf;
  unsigned int robotBodyIndex;

  sva::PTransformd targetTf;

  std::string bodyId;
  Eigen::MatrixXd dofMat;
  Eigen::VectorXd speedMat;
  Eigen::Vector3d normal;

  double stiffness_;
  double weight_;
  double speed_;
  double direction_;
  Eigen::Vector3d targetSpeed;
  std::shared_ptr<tasks::qp::LinVelocityTask> linVelTask;
  std::shared_ptr<tasks::qp::PIDTask> linVelTaskPid;
  double targetVelWeight;

private:
  /* Hide these virtual functions */
  void selectActiveJoints(mc_solver::QPSolver &,
                          const std::vector<std::string> &,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & = {}) override
  {
  }

  void selectUnactiveJoints(mc_solver::QPSolver &,
                            const std::vector<std::string> &,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & = {}) override
  {
  }

  void resetJointsSelector(mc_solver::QPSolver &) override {}

  void reset() override {}

private:
  void addToSolver(mc_solver::QPSolver & solver) override;

  void removeFromSolver(mc_solver::QPSolver & solver) override;

  void update(mc_solver::QPSolver &) override;

  bool manageConstraint = false;
};

/*! \brief Add a contact
 *
 * This is simply a special case of the generic AddRemoveContactTask
 * where direction is equal to -1
 *
 */
struct MC_TASKS_DLLAPI AddContactTask : public AddRemoveContactTask
{
public:
  /*! \brief Constructor */
  AddContactTask(mc_rbdyn::Robots & robots,
                 std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                 mc_rbdyn::Contact & contact,
                 double speed = 0.01,
                 double stiffness = 2,
                 double weight = 1000,
                 Eigen::Vector3d * userT_0_s = nullptr);

  /*! \brief Constructor (self-managed speed constraint) */
  AddContactTask(mc_solver::QPSolver & solver,
                 mc_rbdyn::Contact & contact,
                 double speed = 0.01,
                 double stiffness = 2,
                 double weight = 1000,
                 Eigen::Vector3d * userT_0_s = nullptr);
};

/*! \brief Remove a contact
 *
 * This is simply a special case of the generic AddRemoveContactTask
 * where direction is equal to 1
 *
 */
struct MC_TASKS_DLLAPI RemoveContactTask : public AddRemoveContactTask
{
public:
  /*! \brief Constructor */
  RemoveContactTask(mc_rbdyn::Robots & robots,
                    std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                    mc_rbdyn::Contact & contact,
                    double speed = 0.01,
                    double stiffness = 2,
                    double weight = 1000,
                    Eigen::Vector3d * userT_0_s = nullptr);

  /*! \brief Constructor (self-managed speed constraint) */
  RemoveContactTask(mc_solver::QPSolver & solver,
                    mc_rbdyn::Contact & contact,
                    double speed = 0.01,
                    double stiffness = 2,
                    double weight = 1000,
                    Eigen::Vector3d * userT_0_s = nullptr);
};

} // namespace mc_tasks

#endif
