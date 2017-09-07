#pragma once

#include <mc_control/mc_controller.h>

#include <mc_rbdyn/stance.h>
#include <mc_rbdyn/polygon_utils.h>
#include <mc_tasks/StabilityTask.h>
#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/MoveContactTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_solver/BoundedSpeedConstr.h>
#include <mc_solver/CoMIncPlaneConstr.h>
#include <Tasks/QPConstr.h>

#include <mc_control/ContactSensor.h>

#include <boost/timer/timer.hpp>

#include <mc_control/api.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI ActiGripper
{
public:
  ActiGripper();

  ActiGripper(const std::string& wrenchName, double actiForce, double stopForce,
              double actiTorque, double stopTorque,
              tasks::qp::ContactId contactId, sva::PTransformd & X_0_s,
              double maxDist, double maxRot,
              const std::shared_ptr<tasks::qp::PositionTask> & positionTask,
              const std::shared_ptr<tasks::qp::SetPointTask> & positionTaskSp,
              const std::shared_ptr<tasks::qp::OrientationTask> & orientationTask,
              const std::shared_ptr<tasks::qp::SetPointTask> & orientationTaskSp);

  /** \brief Update the position task reference according to the forceNorm measured
   * on the sensor.
   *
   * This method will update the position task by following an impedance control
   * law. It will add and remove dofs on the selected contact when activating
   * or deactivating. If the error is too big, an emergency stop should be
   * triggered on the FSM side.
   * @return bool: true if update is successful, false if the error is too big.
  **/
 bool update(const mc_rbdyn::Robot & robot,
      tasks::qp::ContactConstr* contactConstr);

protected:
 bool updateForce(double forceNorm, tasks::qp::ContactConstr* contactConstr);
 bool updateTorque(double torqueNorm, tasks::qp::ContactConstr* contactConstr);

public:
  std::string wrenchName;
  double actiForce;
  double stopForce;
  double actiTorque;
  double stopTorque;
  tasks::qp::ContactId contactId;
  sva::PTransformd X_0_s;
  double maxDist;
  double maxRot;
  std::shared_ptr<tasks::qp::PositionTask> positionTask;
  std::shared_ptr<tasks::qp::SetPointTask> positionTaskSp;
  std::shared_ptr<tasks::qp::OrientationTask> orientationTask;
  std::shared_ptr<tasks::qp::SetPointTask> orientationTaskSp;
  bool activatedForce;
  bool activatedTorque;
  Eigen::Vector3d zVec;
  bool toRemove;
  double targetError;
  Eigen::MatrixXd dof;
};

struct MC_CONTROL_DLLAPI CollisionPair
{
public:
  CollisionPair(const mc_rbdyn::Robot & r1, const mc_rbdyn::Robot & r2,
                const std::string & r1bodyName, const std::string & r2bodyName);

  double distance(const mc_rbdyn::Robot & r1, const mc_rbdyn::Robot & r2);
public:
  unsigned int r1BodyIndex;
  unsigned int r2BodyIndex;
  std::shared_ptr<sch::S_Polyhedron> r1hull;
  std::shared_ptr<sch::S_Polyhedron> r2hull;
  sva::PTransformd X_b1_h1;
  sva::PTransformd X_b2_h2;
  std::shared_ptr<sch::CD_Pair> pair;
private:
  void setTransform(const mc_rbdyn::Robot & r1, const mc_rbdyn::Robot & r2);
};

struct SeqAction;

MC_CONTROL_DLLAPI std::vector<mc_rbdyn::Collision> confToColl(const std::vector<mc_rbdyn::StanceConfig::BodiesCollisionConf> & conf);

struct MC_CONTROL_DLLAPI MCSeqTimeLog
{
public:
  MCSeqTimeLog(double timeStep);

  void logPhase(const std::string & phase, uint64_t iter);

  void report();

  void report(const std::string & file);
private:
  void report(std::ostream & os);
  double timeStep;
  std::vector<std::string> phases;
  std::vector<uint64_t> iters;
};

/* Used to publish debug information specific to the seq controller */
struct MCSeqPublisher;

struct MC_CONTROL_DLLAPI MCSeqControllerConfig
{
  MCSeqControllerConfig(const mc_control::Configuration & conf);
  std::shared_ptr<mc_rbdyn::RobotModule> env_module;
  std::string plan;
  bool step_by_step = false;
  bool is_simulation = false;
  bool use_real_sensors = false;
  unsigned int start_stance = 0;
};

struct MC_CONTROL_DLLAPI MCSeqController : public MCController
{
public:
  MCSeqController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt, const MCSeqControllerConfig & config);

  MCSeqController(const MCSeqController&) = delete;
  MCSeqController& operator=(const MCSeqController&) = delete;

  virtual bool run() override;

  virtual void reset(const ControllerResetData & reset_data) override;

  virtual std::vector<std::string> supported_robots() const override;
  /* Utils functions called by SeqStep/SeqAction */
  void updateRobotEnvCollisions(const std::vector<mc_rbdyn::Contact> & contacts, const mc_rbdyn::StanceConfig & conf);

  void updateSelfCollisions(const std::vector<mc_rbdyn::Contact> & contacts, const mc_rbdyn::StanceConfig & conf);

  void updateContacts(const std::vector<mc_rbdyn::Contact> & contacts);

  void pre_live();

  void post_live();

  mc_rbdyn::StanceConfig & curConf();

  mc_rbdyn::Stance & curStance();

  mc_rbdyn::StanceAction & curAction();

  mc_rbdyn::StanceConfig & targetConf();

  mc_rbdyn::Stance & targetStance();

  mc_rbdyn::StanceAction & targetAction();

  std::vector<std::string> bodiesFromContacts(const mc_rbdyn::Robot & robot, const std::vector<mc_rbdyn::Contact> & robotContacts);

  std::vector< std::pair<std::string, std::string> > collisionsContactFilterList(const mc_rbdyn::Contact & contact, const mc_rbdyn::StanceConfig & conf);

  bool setCollisionsContactFilter(const mc_rbdyn::Contact & contact, const mc_rbdyn::StanceConfig & conf);

  bool inContact(const std::string & sname);

  void removeMetaTask(mc_tasks::MetaTask* mt);
  /* Services */
  virtual bool play_next_stance() override;
  virtual bool set_joint_pos(const std::string & jname, const double & pos) override;
  /* Configuration */
  void loadStanceConfigs(const std::string & file);
public:
  /* For logging */
  uint64_t nrIter;
  MCSeqTimeLog time_logger;
  /* Publish information */
  std::shared_ptr<MCSeqPublisher> publisher;
  /* Sequence playing logic */
  bool step_by_step;
  bool paused;
  bool halted;
  unsigned int stanceIndex;
  std::vector<mc_rbdyn::Stance> stances;
  std::vector<mc_rbdyn::StanceConfig> configs;
  std::vector<mc_rbdyn::PolygonInterpolator> interpolators;
  double interpol_percent;
  std::vector< std::shared_ptr<mc_rbdyn::StanceAction> > actions;
  std::vector< std::shared_ptr<SeqAction> > seq_actions;
  std::vector<mc_tasks::MetaTask*> metaTasks;
  std::map<std::string, ActiGripper> actiGrippers;
  std::vector< std::shared_ptr<CollisionPair> > distPairs;
  Eigen::Vector3d contactPos;
  mc_rbdyn::Contact * currentContact;
  mc_rbdyn::Contact * targetContact;
  mc_control::Gripper * currentGripper;
  bool currentGripperIsClosed;
  Eigen::Vector3d errorI;
  Eigen::Vector3d oriErrorI;
  bool isColl;
  bool isCollFiltered;
  bool push;
  unsigned int notInContactCount;
  bool isGripperOpen;
  bool isGripperClose;
  bool isGripperAttached;
  bool isGripperWillBeAttached;
  bool isRemoved;
  bool isBodyTask;
  bool isAdjust;
  bool skipRemoveGripper;
  std::vector<std::string> lowerPGainsJoints;
  std::vector<double> lowerPGainsOriginalValues;
  unsigned int iterSinceHardClose;
  bool comRemoveGripper;

  /* Contact sensors */
  bool is_simulation;
  bool use_real_sensors;
  std::shared_ptr<ContactSensor> contactSensor; /* Update the surfaces that are in contact */
  std::vector<std::string> sensorContacts; /* Contain the name of surfaces in contact */

  /* Tasks and constraints specific to seq controller */
  mc_solver::RobotEnvCollisionsConstraint collsConstraint;
  std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr;
  std::shared_ptr<mc_tasks::StabilityTask> stabilityTask;
  std::shared_ptr<mc_tasks::MoveContactTask> moveContactTask;
  std::shared_ptr<mc_tasks::AddContactTask> addContactTask;
  std::shared_ptr<mc_tasks::RemoveContactTask> rmContactTask;
  std::shared_ptr<mc_tasks::RemoveContactTask> removeContactTask;
  std::shared_ptr<tasks::qp::OrientationTask> bodyOriTask;
  std::shared_ptr<tasks::qp::SetPointTask> bodyOriTaskSp;
  std::shared_ptr<tasks::qp::PositionTask> adjustPositionTask;
  std::shared_ptr<tasks::qp::OrientationTask> adjustOrientationTask;
  std::shared_ptr<tasks::qp::PIDTask> adjustPositionTaskPid;
  std::shared_ptr<tasks::qp::PIDTask> adjustOrientationTaskPid;
  std::shared_ptr<mc_tasks::ComplianceTask> complianceTask;
  std::vector< std::shared_ptr<tasks::qp::GripperTorqueTask> > gripperTorqueTasks;
  mc_solver::CoMIncPlaneConstr comIncPlaneConstr;
  bool startPolygonInterpolator = false;
  double max_perc;
  unsigned int nr_points;
  mc_rbdyn::QuadraticGenerator samples;
  std::vector<mc_rbdyn::Plane> planes;
  boost::timer::cpu_timer timer;
};

MC_CONTROL_DLLAPI std::shared_ptr<SeqAction> seqActionFromStanceAction(mc_rbdyn::StanceAction * curAction, mc_rbdyn::StanceAction * targetAction, mc_rbdyn::StanceAction * targetTargetAction);

struct SeqStep;

struct MC_CONTROL_DLLAPI SeqAction
{
public:
  enum SeqActionType
  {
    CoMMove = 1,
    ContactMove = 2,
    GripperMove = 3
  };
public:
  SeqAction();

  virtual bool execute(MCSeqController & controller);

  SeqActionType type() const;
public:
  SeqActionType _type;
  unsigned int currentStep;
  std::vector< std::shared_ptr<SeqStep> > steps;
};

struct MC_CONTROL_DLLAPI SeqStep
{
public:
  virtual bool eval(MCSeqController & controller);
};

}

extern "C"
{
  MC_CONTROL_DLLAPI std::vector<std::string> MC_RTC_CONTROLLER();
  MC_CONTROL_DLLAPI void destroy(mc_control::MCController * ptr);
  MC_CONTROL_DLLAPI mc_control::MCController * create(const std::string &, const std::shared_ptr<mc_rbdyn::RobotModule> & robot, const double & dt, const mc_control::Configuration & conf);
}
