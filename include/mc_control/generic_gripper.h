#ifndef _H_GENERICGRIPPER_H_
#define _H_GENERICGRIPPER_H_

#include <map>
#include <string>
#include <vector>

#include <mc_rbdyn/robot.h>

namespace mc_control
{

struct Mimic
{
public:
  Mimic();
  Mimic(const std::string & j, double m, double o);
public:
  std::string joint;
  double multiplier;
  double offset;
};

typedef std::map<std::string, Mimic> mimic_d_t;

mimic_d_t readMimic(const std::string & urdf);

int findSuccessorJoint(const mc_rbdyn::Robot & robot, int bodyIndex);

std::vector<std::string> gripperJoints(const mc_rbdyn::Robot & robot, const std::string & gripperName, const mimic_d_t & mimicDict);

std::string findFirstCommonBody(const mc_rbdyn::Robot & robotFull, const std::string & bodyName, const mc_rbdyn::Robot & robot);

struct Gripper
{
public:
  Gripper(const mc_rbdyn::Robot & robot, const std::string & gripperName,
          const mc_rbdyn::Robot & controlRobot, const std::string & robot_urdf,
          double currentQ, double timeStep);

  void setCurrentQ(double currentQ);

  void setTargetQ(double targetQ);

  void setTargetOpening(double targetOpening);

  double curPosition() const;

  const std::vector<double> & q();

  void setActualQ(double q);
public:
  std::vector<std::string> name;

  double closeP;
  double openP;
  double vmax;
  double timeStep;
  double percentOpen;

  std::vector<double> mult;
  std::vector<double> _q;

  std::string controlBodyName;
  double targetQIn;
  double * targetQ;

  /* True if the gripper has been too far from the command for over overCommandLimitIterN iterations */
  bool overCommandLimit;
  unsigned int overCommandLimitIter;
  unsigned int overCommandLimitIterN;
  double actualQ;
  double actualCommandDiffTrigger;
};

}

#endif
