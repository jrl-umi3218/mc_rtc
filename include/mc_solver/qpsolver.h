#ifndef _H_MCCONTROLQPSOLVER_H_
#define _H_MCCONTROLQPSOLVER_H_

#include <mc_control/msg/QPResult.h>
#include <mc_control/msg/Contact.h>
#include <mc_control/msg/Robot.h>
#include <mc_control/msg/QPResult.h>

#include <mc_rbdyn/Collision.h>
#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/robot.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <Tasks/QPConstr.h>
#include <Tasks/QPContactConstr.h>
#include <Tasks/QPMotionConstr.h>
#include <Tasks/QPSolver.h>

#include <memory>
#include <set>

#include <mc_solver/api.h>

namespace mc_control
{
  struct MCController;
}

namespace mc_solver
{

struct QPSolver;

typedef std::pair<std::string, std::function<void (rbd::MultiBody&, rbd::MultiBodyConfig&, tasks::qp::QPSolver&)> > qpcallback_t;

bool operator==(const qpcallback_t & lhs, const qpcallback_t & rhs);

bool operator!=(const qpcallback_t & lhs, const qpcallback_t & rhs);

/* Note: a tasks::qp::Constraint is actually not very useful, we will need to
 * cast this later */
struct MC_SOLVER_DLLAPI ConstraintSet
{
public:
  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const = 0;

  virtual void removeFromSolver(tasks::qp::QPSolver & solver) const = 0;

  std::vector<qpcallback_t> preQPCb;
  std::vector<qpcallback_t> postQPCb;

  virtual ~ConstraintSet() {}
};

struct MC_SOLVER_DLLAPI ContactConstraint : public ConstraintSet
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

  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const override;

  virtual void removeFromSolver(tasks::qp::QPSolver & solver) const override;
public:
  std::shared_ptr<tasks::qp::ContactConstr> contactConstr;
  std::shared_ptr<tasks::qp::PositiveLambda> posLambdaConstr;
public:
  ContactConstraint() {}
};

struct MC_SOLVER_DLLAPI KinematicsConstraint : public ConstraintSet
{
public:
  KinematicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep, bool isStatic = false,
                       const std::vector<double> & damper = {}, double velocityPercent = 1.0);

  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const override;

  virtual void removeFromSolver(tasks::qp::QPSolver & solver) const override;
public:
  /* This one actually holds a tasks::qp::JointLimitsConstr or a tasks::qp::DamperJointLimitsConstr */
  std::shared_ptr<tasks::qp::JointLimitsConstr> jointLimitsConstr;
  std::shared_ptr<tasks::qp::DamperJointLimitsConstr> damperJointLimitsConstr;
  bool damped;
public:
  KinematicsConstraint() {}
};

struct MC_SOLVER_DLLAPI DynamicsConstraint : public KinematicsConstraint
{
public:
  DynamicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep, bool isStatic = false,
                     const std::vector<double> & damper = {}, double velocityPercent = 1.0, bool infTorque = false);

  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const override;

  virtual void removeFromSolver(tasks::qp::QPSolver & solver) const override;
public:
  /* This one actually holds a tasks::qp::MotionSpringConstr, a tasks::qp::MotionPolyConstr or a tasks::qp::MotionConstr */
  std::shared_ptr<tasks::qp::MotionConstr> motionConstr;
  std::shared_ptr<tasks::qp::MotionSpringConstr> motionSpringConstr;
  bool is_spring;
  bool is_poly;
public:
  DynamicsConstraint() {}
};

struct MC_SOLVER_DLLAPI CollisionsConstraint : public ConstraintSet
{
public:
  constexpr static double defaultDampingOffset = 0.1;
public:
  CollisionsConstraint(const mc_rbdyn::Robots & robots, unsigned int r1Index, unsigned int r2Index, double timeStep);

  bool removeCollision(QPSolver & solver, const std::string & b1Name, const std::string & b2Name);

  bool removeCollisionByBody(QPSolver & solver, const std::string & byName, const std::string & b2Name);

  void addCollision(QPSolver & solver, const mc_rbdyn::Collision & col);

  void addCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Collision> & cols);

  void reset();

  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const override;

  virtual void removeFromSolver(tasks::qp::QPSolver & solver) const override;
public:
  std::shared_ptr<tasks::qp::CollisionConstr> collConstr;
  unsigned int r1Index;
  unsigned int r2Index;
  std::vector<mc_rbdyn::Collision> cols;
private:
  int collId;
  std::map<std::string, std::pair<unsigned int, mc_rbdyn::Collision> > collIdDict;
  std::string __keyByNames(const std::string & name1, const std::string & name2);
  int __createCollId(const mc_rbdyn::Collision & col);
  std::pair<int, mc_rbdyn::Collision> __popCollId(const std::string & name1, const std::string & name2);
  void __addCollision(const mc_rbdyn::Robots & robots, const mc_rbdyn::Collision & col);
public:
  CollisionsConstraint() {}
};

struct MC_SOLVER_DLLAPI RobotEnvCollisionsConstraint : public ConstraintSet
{
public:
  RobotEnvCollisionsConstraint(const mc_rbdyn::Robots & robots, double timeStep);

  bool removeEnvCollision(QPSolver & solver, const std::string & rBodyName, const std::string & eBodyName);

  bool removeEnvCollisionByBody(QPSolver & solver, const std::string & rBodyName, const std::string & eBodyName);

  bool removeSelfCollision(QPSolver & solver, const std::string & body1Name, const std::string & body2Name);

  void addEnvCollision(QPSolver & solver, const mc_rbdyn::Collision & col);

  void addSelfCollision(QPSolver & solver, const mc_rbdyn::Collision & col);

  void setEnvCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Contact> & contacts,
                                                         const std::vector<mc_rbdyn::Collision> & cols);

  void setSelfCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Contact> & contacts,
                                                         const std::vector<mc_rbdyn::Collision> & cols);

  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const override;

  virtual void removeFromSolver(tasks::qp::QPSolver & solver) const override;
public:
  CollisionsConstraint selfCollConstrMng;
  CollisionsConstraint envCollConstrMng;
  /* Note this class maintains its constraints member as the concatenation of its two CollisionsConstraint constraints */
private:
  std::set<int> __bodiesFromContacts(const mc_rbdyn::Robot & robot, const std::vector<mc_rbdyn::Contact> & contacts);
};

struct MC_SOLVER_DLLAPI QPSolver
{
public:
  QPSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double timeStep);

  void addConstraintSet(const ConstraintSet & cs);

  void removeConstraintSet(const ConstraintSet & cs);

  void addTask(tasks::qp::Task * task);

  void removeTask(tasks::qp::Task * task);

  template<typename ... Fun>
  void addConstraint(tasks::qp::ConstraintFunction<Fun...> * constraint)
  {
    constraint->addToSolver(robots().mbs(), solver);
  }

  template<typename ... Fun>
  void removeConstraint(tasks::qp::ConstraintFunction<Fun...> * constraint)
  {
    constraint->removeFromSolver(solver);
  }

  std::pair<int, const tasks::qp::BilateralContact&> contactById(const tasks::qp::ContactId & id);

  void setContacts(const std::vector<mc_rbdyn::Contact> & contacts);

  bool run();

  const mc_control::QPResultMsg & send(double curTime = 0);

  const mc_rbdyn::Robot & robot() const;
  mc_rbdyn::Robot & robot();

  const mc_rbdyn::Robot & env() const;
  mc_rbdyn::Robot & env();

  const mc_rbdyn::Robots & robots() const;
  mc_rbdyn::Robots & robots();

  void updateConstrSize();
private:
  std::shared_ptr<mc_rbdyn::Robots> robots_p;
  double timeStep;
  std::vector<qpcallback_t> preQPCb;
  std::vector<qpcallback_t> postQPCb;

  std::vector<tasks::qp::UnilateralContact> uniContacts;
  std::vector<tasks::qp::BilateralContact> biContacts;

  tasks::qp::QPSolver solver;
  mc_control::QPResultMsg qpRes;

  void __fillResult();
public:
  QPSolver() {}
};

}

#endif
