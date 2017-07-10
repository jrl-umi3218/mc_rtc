#ifndef _H_ADDREMOVECONTACTTASK_H_
#define _H_ADDREMOVECONTACTTASK_H_

#include <mc_tasks/MetaTask.h>

#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/StanceConfig.h>

#include <mc_solver/BoundedSpeedConstr.h>

#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>

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
  /*! \brief Constructor based on mc_rbdyn::StanceConfig
   *
   * Constructs an AddRemoveContactTask using information stored in a
   * mc_rbdyn::StanceConfig value and other elements
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
   * \param config Use config.contactTask.linVel for configuration
   * purposes
   *
   * \param userT_0_s If provided, overrides the chosen normal
   * direction
   */
  AddRemoveContactTask(mc_rbdyn::Robots & robots,
                       std::shared_ptr<mc_solver::BoundedSpeedConstr>
                       constSpeedConstr, mc_rbdyn::Contact & contact,
                       double direction, const mc_rbdyn::StanceConfig &
                       config, Eigen::Vector3d * userT_0_s = nullptr);

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
                       std::shared_ptr<mc_solver::BoundedSpeedConstr>
                       constSpeedConstr, mc_rbdyn::Contact & contact,
                       double direction, double speed,
                       double stiffness, double weight,
                       Eigen::Vector3d * userT_0_s = nullptr);

  /*! \brief General constructor with self-managed bounded speed constraint
   *
   * Constructs an AddRemoveContactTask using information passed by the programmer. No mc_solver::BoundedSpeedConstr has to be passed. Instead the constraint is created and managed by the task.
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
                       double direction, double speed,
                       double stiffness, double weight,
                       Eigen::Vector3d * userT_0_s = nullptr);

  /*! \brief Set the displacement direction
   *
   * \param direction Direction of the displacement
   *
   */
  void direction(double direction);

  /*! \brief Get the desired dislacement speed */
  double speed() { return speed_; }
  /*! \brief Set the desired dislacement speed */
  void speed(double s) { speed_ = s; }

  /*! \brief Get the task stiffness */
  double stiffness() { return stiffness_; }
  /*! \brief Get the task weight */
  double weight() { return weight_; }

  /*! \brief Get the velocity error
   *
   * \returns Velocity error of the LinVelocity task */
  Eigen::Vector3d velError();

  virtual void dimWeight(const Eigen::VectorXd & dimW) override;

  virtual Eigen::VectorXd dimWeight() const override;

  virtual Eigen::VectorXd eval() const override;

  virtual Eigen::VectorXd speed() const override;
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
  Eigen::Vector3d targetSpeed;
  std::shared_ptr<tasks::qp::LinVelocityTask> linVelTask;
  std::shared_ptr<tasks::qp::PIDTask> linVelTaskPid;
  double targetVelWeight;
private:
  /* Hide these virtual functions */
  virtual void selectActiveJoints(mc_solver::QPSolver &,
                                  const std::vector<std::string> &) override {}

  virtual void selectUnactiveJoints(mc_solver::QPSolver &,
                                    const std::vector<std::string> &) override {}

  virtual void resetJointsSelector(mc_solver::QPSolver &) override {}

  virtual void reset() override {}
private:
  virtual void addToSolver(mc_solver::QPSolver & solver) override;

  virtual void removeFromSolver(mc_solver::QPSolver & solver) override;

  virtual void update() override;

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
  /*! \brief Constructor (mc_rbdyn::StanceConfig variant) */
  AddContactTask(mc_rbdyn::Robots & robots,
                 std::shared_ptr<mc_solver::BoundedSpeedConstr>
                 constSpeedConstr, mc_rbdyn::Contact & contact, const
                 mc_rbdyn::StanceConfig & config, Eigen::Vector3d *
                 userT_0_s = nullptr);

  /*! \brief Constructor */
  AddContactTask(mc_rbdyn::Robots & robots,
                 std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                 mc_rbdyn::Contact & contact,
                 double speed, double stiffness, double weight,
                 Eigen::Vector3d * userT_0_s = nullptr);

  /*! \brief Constructor (self-managed speed constraint) */
  AddContactTask(mc_solver::QPSolver & solver,
                 mc_rbdyn::Contact & contact,
                 double speed, double stiffness, double weight,
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
  /*! \brief Constructor (mc_rbdyn::StanceConfig variant) */
  RemoveContactTask(mc_rbdyn::Robots & robots,
                    std::shared_ptr<mc_solver::BoundedSpeedConstr>
                    constSpeedConstr, mc_rbdyn::Contact & contact,
                    const mc_rbdyn::StanceConfig & config,
                    Eigen::Vector3d * userT_0_s = nullptr);

  /*! \brief Constructor */
  RemoveContactTask(mc_rbdyn::Robots & robots,
                 std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                 mc_rbdyn::Contact & contact,
                 double speed, double stiffness, double weight,
                 Eigen::Vector3d * userT_0_s = nullptr);

  /*! \brief Constructor (self-managed speed constraint) */
  RemoveContactTask(mc_solver::QPSolver & solver,
                    mc_rbdyn::Contact & contact,
                    double speed, double stiffness, double weight,
                    Eigen::Vector3d * userT_0_s = nullptr);
};

}

#endif
