#ifndef _H_MCVIRTUALCONTROLLER_H_
#define _H_MCVIRTUALCONTROLLER_H_

#include <mc_rbdyn/robot.h>
#include <mc_control/generic_gripper.h>
#include <mc_control/msg/QPResult.h>

#include <Tasks/QPTasks.h>

namespace mc_rbdyn
{
  struct Contact;
}

namespace mc_control
{

/* This structure will be filled in time with the necessary information */
/* Coming most likely from the previous controller */
struct ControllerResetData
{
  const std::vector< std::vector<double> > & q;
};

struct MCGlobalController;

/*FIXME Get some data as parameters (e.g. timeStep, path to default env...) */
struct MCVirtualController
{
  friend struct MCGlobalController;
public:
  MCVirtualController();

  virtual bool run() = 0;

  virtual const QPResultMsg & send(const double & t) = 0;

  virtual void reset(const ControllerResetData & reset_data) = 0;

  virtual void setWrenches(const std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > & wrenches) = 0;
  /* Helper function to access robots, robot and env */
  virtual const mc_rbdyn::Robot & robot() const = 0;

  virtual const mc_rbdyn::Robot & env() const = 0;

  virtual const mc_rbdyn::Robots & robots() const = 0;

  virtual mc_rbdyn::Robots & robots() = 0;

  virtual mc_rbdyn::Robot & robot() = 0;

  virtual mc_rbdyn::Robot & env() = 0;
  /* Joint control service available to all controllers */
  virtual bool joint_up(const std::string & jname) = 0;
  virtual bool joint_down(const std::string & jname) = 0;

  virtual bool set_joint_pos(const std::string & jname, const double & pos) = 0;

  /* Generic message passing service */
  virtual bool read_msg(std::string & msg);

  virtual bool read_write_msg(std::string & msg, std::string & out);
public:
  /* Common stuff */
  const double timeStep;
  /* Encoder values provided by the low-level controller */
  std::vector<double> encoderValues;
  /* Robot orientation provided by sensors */
  Eigen::Vector3d sensorOri;
  /* FIXME A bit dirty but needed */
  std::shared_ptr<mc_control::Gripper> lgripper;
  std::shared_ptr<mc_control::Gripper> rgripper;
};

}

#endif
