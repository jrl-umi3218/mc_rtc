#ifndef _H_MCSEQCONTROLLER_H_
#define _H_MCSEQCONTROLLER_H_

#include <mc_control/mc_controller.h>

#include <mc_rbdyn/stance.h>
#include <mc_rbdyn/polygon_utils.h>
#include <mc_tasks/StabilityTask.h>
#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/MoveContactTask.h>
#include <Tasks/QPConstr.h>

#include <mc_control/ContactSensor.h>

namespace mc_control
{

struct ActiGripper
{
public:
  ActiGripper();

  ActiGripper(unsigned int wrenchIndex, double actiForce, double stopForce,
              tasks::qp::ContactId contactId, sva::PTransformd & X_0_s, double maxDist,
              const std::shared_ptr<tasks::qp::PositionTask> & positionTask,
              const std::shared_ptr<tasks::qp::SetPointTask> & positionTaskSp);
public:
  unsigned int wrenchIndex;
  double actiForce;
  double stopForce;
  tasks::qp::ContactId contactId;
  sva::PTransformd X_0_s;
  double maxDist;
  std::shared_ptr<tasks::qp::PositionTask> positionTask;
  std::shared_ptr<tasks::qp::SetPointTask> positionTaskSp;
  bool activated;
  Eigen::Vector3d zVec;
  bool toRemove;
  double targetError;
};

struct CollisionPair
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

std::vector<mc_solver::Collision> confToColl(const std::vector<mc_rbdyn::StanceConfig::BodiesCollisionConf> & conf);

struct MCSeqTimeLog
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

struct MCSeqController : public MCController
{
public:
  MCSeqController(const std::string & env_name, const std::string & seq_path, bool real_sensors, unsigned int start_stance, bool step_by_step);

  MCSeqController(const std::string & env_path, const std::string & env_name, const std::string & seq_path, bool real_sensors, unsigned int start_stance, bool step_by_step);

  MCSeqController(const std::shared_ptr<mc_rbdyn::RobotModule> & env_module, const std::string & seq_path, bool real_sensors, unsigned int start_stance, bool step_by_step);

  virtual bool run() override;

  virtual void reset(const ControllerResetData & reset_data) override;
  /* Utils functions called by SeqStep/SeqAction */
  void updateRobotEnvCollisions(const std::vector<mc_rbdyn::Contact> & contacts, const mc_rbdyn::StanceConfig & conf);

  void updateSelfCollisions(const std::vector<mc_rbdyn::Contact> & contacts, const mc_rbdyn::StanceConfig & conf);

  void updateContacts(const std::vector<mc_rbdyn::Contact> & contacts);

  void updateSolverEqInEq();

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
  bool play_next_stance();
  /* Configuration */
  void loadStanceConfigs(const std::string & file);
public:
  /* For logging */
  uint64_t nrIter;
  MCSeqTimeLog logger;
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
  bool use_real_sensors; /*FIXME Should be set by configuration */
  std::shared_ptr<ContactSensor> contactSensor; /* Update the surfaces that are in contact */
  std::vector<std::string> sensorContacts; /* Contain the name of surfaces in contact */

  /* Tasks and constraints specific to seq controller */
  mc_solver::RobotEnvCollisionsConstraint collsConstraint;
  std::shared_ptr<tasks::qp::BoundedSpeedConstr> constSpeedConstr;
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
  std::vector< std::shared_ptr<tasks::qp::GripperTorqueTask> > gripperTorqueTasks;
  std::shared_ptr<tasks::qp::CoMIncPlaneConstr> comIncPlaneConstr;
  double max_perc;
  unsigned int nr_points;
  mc_rbdyn::QuadraticGenerator samples;
  std::vector<mc_rbdyn::Plane> planes;
};

std::shared_ptr<SeqAction> seqActionFromStanceAction(mc_rbdyn::StanceAction * curAction, mc_rbdyn::StanceAction * targetAction, mc_rbdyn::StanceAction * targetTargetAction);

struct SeqStep;

struct SeqAction
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

struct SeqStep
{
public:
  virtual bool eval(MCSeqController & controller);
};

}

#endif
