#ifndef _H_MCCONTROLQPSOLVER_H_
#define _H_MCCONTROLQPSOLVER_H_

#include <mc_control/msg/QPResult.h>
#include <mc_control/msg/Contact.h>
#include <mc_control/msg/Robot.h>
#include <mc_control/msg/MRQPResult.h>

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/robot.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <Tasks/QPSolver.h>

#include <memory>
#include <set>

namespace mc_control
{
  struct MCController;
}

namespace mc_solver
{

typedef std::pair<std::string, std::function<void (rbd::MultiBody&, rbd::MultiBodyConfig&, tasks::qp::QPSolver&)> > qpcallback_t;

bool operator==(const qpcallback_t & lhs, const qpcallback_t & rhs);

bool operator!=(const qpcallback_t & lhs, const qpcallback_t & rhs);

/* Note: a tasks::qp::Constraint is actually not very useful, we will need to
 * cast this later */
struct ConstraintSet
{
public:
  std::vector<std::shared_ptr<tasks::qp::Constraint> > constraints;
  std::vector<qpcallback_t> preQPCb;
  std::vector<qpcallback_t> postQPCb;
};

struct ContactConstraint : public ConstraintSet
{
public:
  enum ContactType
  {
    Acceleration = 0,
    Velocity = 1,
    Position = 2
  };
public:
  ContactConstraint(double timeStep, ContactType contactType= Velocity, bool dynamics = true);
public:
  /* This one actually holds a tasks::qp::ContactConstraint */
  std::shared_ptr<tasks::qp::Constraint> contactConstr;
public:
  ContactConstraint() {}
};

struct KinematicsConstraint : public ConstraintSet
{
public:
  KinematicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep, bool isStatic = false,
                       const std::vector<double> & damper = {}, double velocityPercent = 1.0);
public:
  /* This one actually holds a tasks::qp::JointLimitsConstr or a tasks::qp::DamperJointLimitsConstr */
  std::shared_ptr<tasks::qp::Constraint> jointLimitsConstr;
  bool damped;
public:
  KinematicsConstraint() {}
};

struct DynamicsConstraint : public KinematicsConstraint
{
public:
  DynamicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep, bool isStatic = false,
                     const std::vector<double> & damper = {}, double velocityPercent = 1.0, bool infTorque = false);
public:
  /* This one actually holds a tasks::qp::MotionSpringConstr, a tasks::qp::MotionPolyConstr or a tasks::qp::MotionConstr */
  std::shared_ptr<tasks::qp::Constraint> motionConstr;
  bool is_spring;
  bool is_poly;
public:
  DynamicsConstraint() {}
};

struct Collision
{
  Collision() : body1("NONE"), body2("NONE") {}
  Collision(const std::string & b1, const std::string & b2, double i, double s, double d)
  : body1(b1), body2(b2), iDist(i), sDist(s), damping(d)
  {}
  std::string body1;
  std::string body2;
  double iDist;
  double sDist;
  double damping;
  bool isNone() { return body1 == "NONE" and body2 == "NONE"; }
};

bool operator==(const Collision & lhs, const Collision & rhs);

bool operator!=(const Collision & lhs, const Collision & rhs);

struct CollisionsConstraint : public ConstraintSet
{
public:
  static double defaultDampingOffset;
public:
  CollisionsConstraint(const mc_rbdyn::Robots & robots, unsigned int r1Index, unsigned int r2Index, double timeStep);

  bool removeCollision(const mc_rbdyn::Robots & robots, const std::string & b1Name, const std::string & b2Name);

  bool removeCollisionByBody(const mc_rbdyn::Robots & robots, const std::string & byName, const std::string & b2Name);

  void addCollision(const mc_rbdyn::Robots & robots, const Collision & col);

  void addCollisions(const mc_rbdyn::Robots & robots, const std::vector<Collision> & cols);
public:
  /* This actually holds a tasks::qp::CollisionConstr */
  std::shared_ptr<tasks::qp::Constraint> collConstr;
  unsigned int r1Index;
  unsigned int r2Index;
  std::vector<Collision> cols;
  unsigned int collId;
  std::map<std::string, std::pair<unsigned int, Collision> > collIdDict;
private:
  std::string __keyByNames(const std::string & name1, const std::string & name2);
  unsigned int __createCollId(const Collision & col);
  std::pair<unsigned int, Collision> __popCollId(const std::string & name1, const std::string & name2);
public:
  CollisionsConstraint() {}
};

struct RobotEnvCollisionsConstraint : public ConstraintSet
{
public:
  RobotEnvCollisionsConstraint(const mc_rbdyn::Robots & robots, double timeStep);

  bool removeEnvCollision(const mc_rbdyn::Robots & robots, const std::string & rBodyName, const std::string & eBodyName);

  bool removeEnvCollisionByBody(const mc_rbdyn::Robots & robots, const std::string & rBodyName, const std::string & eBodyName);

  bool removeSelfCollision(const mc_rbdyn::Robots & robots, const std::string & body1Name, const std::string & body2Name);

  void addEnvCollision(const mc_rbdyn::Robots & robots, const Collision & col);

  void addSelfCollision(const mc_rbdyn::Robots & robots, const Collision & col);

  void setEnvCollisions(const mc_rbdyn::Robots & robots, const std::vector<mc_rbdyn::Contact> & contacts,
                                                         const std::vector<Collision> & cols);

  void setSelfCollisions(const mc_rbdyn::Robots & robots, const std::vector<mc_rbdyn::Contact> & contacts,
                                                         const std::vector<Collision> & cols);
public:
  const mc_rbdyn::Robot & robot;
  const mc_rbdyn::Robot & env;
  CollisionsConstraint selfCollConstrMng;
  CollisionsConstraint envCollConstrMng;
  /* Note this class maintains its constraints member as the concatenation of its two CollisionsConstraint constraints */
private:
  std::set<unsigned int> __bodiesFromContacts(const mc_rbdyn::Robot & robot, const std::vector<mc_rbdyn::Contact> & contacts);
};

struct QPSolver
{
public:
  QPSolver(const mc_rbdyn::Robots & robots, double timeStep);

  void addConstraintSet(ConstraintSet & cs);

  void removeConstraintSet(ConstraintSet & cs);

  void updateNrVars();

  void setContacts(const std::vector<mc_rbdyn::Contact> & contacts, bool useFkPos = false);

  std::vector<double> unilateralContactForce();

  void update();

  bool run();

  const mc_control::QPResultMsg & send(double curTime = 0, unsigned int stanceIndex = 0);
public:
  mc_rbdyn::Robots robots;
  double timeStep;
  /* FIXME Replace with ConstraintSet ? */
  std::vector<std::shared_ptr<tasks::qp::Constraint> > constraints;
  std::vector<qpcallback_t> preQPCb;
  std::vector<qpcallback_t> postQPCb;

  std::vector<tasks::qp::UnilateralContact> uniContacts;
  std::vector<tasks::qp::BilateralContact> biContacts;

  tasks::qp::QPSolver solver;
  mc_control::QPResultMsg qpRes;
private:
  void __fillResult();
public:
  QPSolver() {}
};

struct MRQPSolver
{
public:
  MRQPSolver(const mc_rbdyn::Robots & robots, double timeStep);

  void addConstraintSet(const ConstraintSet & cs);

  void removeConstraintSet(const ConstraintSet & cs);

  void updateNrVars();

  std::pair<int, const tasks::qp::BilateralContact&> contactById(const tasks::qp::ContactId & id);

  void setContacts(const std::vector<mc_rbdyn::Contact> & contacts);

  void update();

  bool run();

  const mc_control::MRQPResultMsg & send(double curTime = 0);
public:
  mc_rbdyn::Robots robots;
  double timeStep;
  /* FIXME Replace with ConstraintSet ? */
  std::vector<std::shared_ptr<tasks::qp::Constraint> > constraints;
  std::vector<qpcallback_t> preQPCb;
  std::vector<qpcallback_t> postQPCb;

  std::vector<tasks::qp::UnilateralContact> uniContacts;
  std::vector<tasks::qp::BilateralContact> biContacts;

  tasks::qp::QPSolver solver;
  mc_control::MRQPResultMsg qpRes;
private:
  void __fillResult();
public:
  MRQPSolver() {}
};

}

#endif
