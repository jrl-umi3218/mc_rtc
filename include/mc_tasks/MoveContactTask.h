#ifndef _H_MOVECONTACTTASK_H_
#define _H_MOVECONTACTTASK_H_

#include <mc_tasks/MetaTask.h>
#include <mc_tasks/SmoothTask.h>

#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/StanceConfig.h>

#include <Tasks/QPConstr.h>
#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>

namespace mc_rbdyn
{
  struct Contact;
}

namespace mc_tasks
{

/*! \brief Move a robot's surface towards a contact's position.
 *
 * The movement stops when the controlled robot's surface is at a certain
 * distance (based on the parameters given to the task)
 *
 * The normal operation is:
 * - create
 * - add to solver and go to waypoint
 * - monitor until waypoint is reached
 * - go to pre-env target
 * - monitor until pre-env target is reached
 * - remove from solver and use mc_tasks::AddContactTask or
 *   mc_tasks::ComplianceTask to add the contact
 *
 */
struct MC_TASKS_DLLAPI MoveContactTask : public MetaTask
{
public:

  /*! \brief Constructor using mc_rbdyn::StanceConfig for configuration
   *
   * \param robots Robots involved in the task
   *
   * \param robot Actual robot controlled by the task
   *
   * \param env Environment where the contact is established
   *
   * \param config Use config.contactTask and config.contactObj for
   * configuration purposes
   *
   * \param positionWStartPercent Initial percentage of the task's final weight
   *
   */
  MoveContactTask(mc_rbdyn::Robots & robots, mc_rbdyn::Robot & robot,
                  mc_rbdyn::Robot & env, mc_rbdyn::Contact & contact,
                  mc_rbdyn::StanceConfig & config,
                  double positionWStartPercent = 0);

  /*! \brief Constructor
   *
   * \param robots Robots involved in the task
   *
   * \param robot Actual robot controlled by the task
   *
   * \param env Environment where the contact is established
   *
   * \param posStifness Initial stiffness of the position task
   *
   * \param extraPosStiffness Extra stiffness applied to the position task
   * while the task is running
   *
   * \param posWeight Weight of the position task
   *
   * \param oriStiffness Stiffness of the orientation task
   *
   * \param oriWeight Weight of the orientation task
   *
   * \param preContactDist Distance to the contact location reached by toPreEnv
   *
   * \param pos Function used to compute the waypoint based on the initial and
   * final location
   *
   * \param adjustOffset Can be used to apply a position offset to the task
   *
   * \param adjustRPYOffset Can be used to apply an orientation offset to the
   * task
   *
   * \param positionWStartPercent Initial percentage of the task's final weight
   *
   */
  MoveContactTask(mc_rbdyn::Robots & robots, mc_rbdyn::Robot & robot,
                  mc_rbdyn::Robot & env, mc_rbdyn::Contact & contact,
                  double posStiffness, double extraPosStiffness, double posWeight,
                  double oriStiffness, double oriWeight,
                  double preContactDist,
                  mc_rbdyn::WaypointFunction waypointPos = mc_rbdyn::percentWaypoint(0.75, 0.75, 0.75, 0.25),
                  const Eigen::Vector3d & adjustOffset = Eigen::Vector3d::Zero(),
                  const Eigen::Vector3d & adjustRPYOffset = Eigen::Vector3d::Zero(),
                  double positionWStartPercent = 0);

  /*! \brief Change the task objective to a waypoint determined by
   * configuration
   *
   * \param config Used to configure the task
   *
   * \param positionSmoothPercent Reset the task smoothing percentage to the
   * provided value
   *
   */
  void toWaypoint(mc_rbdyn::StanceConfig & config, double positionSmoothPercent = 1);

  /*! \brief Change the task objective to a waypoint determined by configuration
   *
   * \param posStifness Initial stiffness of the position task
   *
   * \param extraPosStiffness Extra stiffness applied to the position task
   * while the task is running
   *
   * \param posWeight Weight of the position task
   *
   * \param oriStiffness Stiffness of the orientation task
   *
   * \param oriWeight Weight of the orientation task
   *
   * \param positionSmoothPercent Reset the task smoothing percentage to the
   * provided value
   *
   */
  void toWaypoint(double posStiffness,
                  double extraPosStiffness, double posWeight,
                  double oriStiffness, double oriWeight,
                  double positionSmoothPercent);

  /*! \brief Change the task objective to a waypoint determined by configuration
   *
   * Does not change the tasks stiffness or weight
   *
   */
  void toWaypoint();

  /*! \brief Change the task objective to a point above the environment target
   *
   * \param config Used to configure the task
   *
   * \param positionSmoothPercent Reset the task smoothing percentage to the
   * provided value
   *
   */
  void toPreEnv(mc_rbdyn::StanceConfig & config, double positionSmoothPercent = 1);

  /*! \brief Change the task objective to a waypoint determined by configuration
   *
   * \param posStifness Initial stiffness of the position task
   *
   * \param extraPosStiffness Extra stiffness applied to the position task
   * while the task is running
   *
   * \param posWeight Weight of the position task
   *
   * \param oriStiffness Stiffness of the orientation task
   *
   * \param oriWeight Weight of the orientation task
   *
   * \param positionSmoothPercent Reset the task smoothing percentage to the
   * provided value
   *
   */
  void toPreEnv(double posStiffness,
                  double extraPosStiffness, double posWeight,
                  double oriStiffness, double oriWeight,
                  double positionSmoothPercent);

  /*! \brief Change the task objective to a point above the environment target
   *
   * Does not change the tasks stiffness or weight
   *
   */
  void toPreEnv();

  /*! \brief Returns the current position of the controlled surface
   *
   * \returns The current position of the controlled surface
   */
  sva::PTransformd robotSurfacePos();

  /*! \brief Returns the current velocity of the controlled surface
   *
   * \returns The current velocity of the controlled surface
   */
  sva::MotionVecd robotSurfaceVel();

  /*! \brief Update the target transformation
   *
   * This can be used to change the target in real-time. For example when the
   * target is updated by sensor inputs.
   *
   * After calling this function you should call either toWaypoint or toPreEnv
   * to apply the change.
   *
   * \param X_target New target transformation
   *
   * \param config Configuration used to configure the task
   *
   */
  void set_target_tf(const sva::PTransformd & X_target, mc_rbdyn::StanceConfig & config);

  /*! \brief Update the target transformation
   *
   * This can be used to change the target in real-time. For example when the
   * target is updated by sensor inputs.
   *
   * After calling this function you should call either toWaypoint or toPreEnv
   * to apply the change.
   *
   * \param X_target New target transformation
   *
   * \param preContactDist Distance to the contact location reached by toPreEnv
   *
   * \param pos Function used to compute the waypoint based on the initial and
   * final location
   *
   * \param adjustOffset Can be used to apply a position offset to the task
   *
   * \param adjustRPYOffset Can be used to apply an orientation offset to the
   * task
   *
   */
  void set_target_tf(const sva::PTransformd & X_target,
                     double preContactDist,
                     mc_rbdyn::WaypointFunction waypointPos = mc_rbdyn::percentWaypoint(0.75, 0.75, 0.75, 0.25),
                     const Eigen::Vector3d & adjustOffset = Eigen::Vector3d::Zero(),
                     const Eigen::Vector3d & adjustRPYOffset = Eigen::Vector3d::Zero());

  virtual void dimWeight(const Eigen::VectorXd & dimW) override;

  virtual Eigen::VectorXd dimWeight() const override;

  virtual void selectActiveJoints(mc_solver::QPSolver & solver,
                                  const std::vector<std::string> & aJN) override;

  virtual void selectUnactiveJoints(mc_solver::QPSolver & solver,
                                    const std::vector<std::string> &uJN) override;

  virtual void resetJointsSelector(mc_solver::QPSolver & solver) override;

  virtual Eigen::VectorXd eval() const override;

  virtual Eigen::VectorXd speed() const override;
private:
  void target(const Eigen::Vector3d & pos, const Eigen::Matrix3d & ori,
              double posStiffness, double extraPosStiffness, double posWeight,
              double oriStiffness, double oriWeight,
              double positionSmoothPercent);

  virtual void reset() override {}
public:
  bool inSolver = false;
  mc_rbdyn::Robots & robots;
  mc_rbdyn::Robot & robot;
  mc_rbdyn::Robot & env;

  std::shared_ptr<mc_rbdyn::Surface> robotSurf;
  std::shared_ptr<mc_rbdyn::Surface> envSurf;

  unsigned int robotBodyIndex;
  unsigned int envBodyIndex;

  sva::PTransformd targetTf;
  Eigen::Vector3d targetPos;
  Eigen::Matrix3d targetOri;

  Eigen::Vector3d normal;
  Eigen::Vector3d preTargetPos;
  Eigen::Vector3d wp;

  double posStiff;
  double extraPosStiff;
  std::shared_ptr<tasks::qp::PositionTask> positionTask = nullptr;
  std::shared_ptr<tasks::qp::JointsSelector> positionJSTask = nullptr;
  std::shared_ptr<tasks::qp::SetPointTask> positionTaskSp = nullptr;
  std::shared_ptr<SmoothTask<Eigen::Vector3d>> positionTaskSm = nullptr;

  std::shared_ptr<tasks::qp::OrientationTask> orientationTask = nullptr;
  std::shared_ptr<tasks::qp::JointsSelector> orientationJSTask = nullptr;
  std::shared_ptr<tasks::qp::SetPointTask> orientationTaskSp = nullptr;
  bool useSmoothTask;
private:
  virtual void addToSolver(mc_solver::QPSolver & solver) override;

  virtual void removeFromSolver(mc_solver::QPSolver & solver) override;

  virtual void update() override;
};

}

#endif
