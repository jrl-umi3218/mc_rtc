#include <mc_control/mc_bci_self_interact_controller.h>

#include <mc_rtc/ros.h>
#ifdef MC_RTC_HAS_ROS
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#endif

namespace
{
  inline std::stringstream& operator>>(std::stringstream & ss, Eigen::Matrix3d & m)
  {
    for(int i = 0; i < 3; ++i)
    {
      for(int j = 0; j < 3; ++j)
      {
        ss >> m(i,j);
      }
    }
    return ss;
  }
  inline std::stringstream& operator>>(std::stringstream & ss, Eigen::Vector3d & v)
  {
    for(int i = 0; i < 3; ++i)
    {
      ss >> v(i);
    }
    return ss;
  }
}

namespace mc_control
{

MCBCISelfInteractController::MCBCISelfInteractController()
: MCController(),
  tf_caster(0), seq(0)
{
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(dynamicsConstraint);
  /* Set more restrictive auto collision constraints */
  selfCollisionConstraint.reset();
  selfCollisionConstraint.addCollisions(qpsolver->robots, {
    mc_solver::Collision("LARM_LINK3", "BODY", 0.1, 0.05, 0.),
    mc_solver::Collision("LARM_LINK4", "BODY", 0.1, 0.05, 0.),
    mc_solver::Collision("LARM_LINK5", "BODY", 0.1, 0.05, 0.),
    mc_solver::Collision("RARM_LINK3", "BODY", 0.1, 0.05, 0.),
    mc_solver::Collision("RARM_LINK4", "BODY", 0.1, 0.05, 0.),
    mc_solver::Collision("RARM_LINK5", "BODY", 0.1, 0.05, 0.),
    mc_solver::Collision("RARM_LINK3", "CHEST_LINK0", 0.1, 0.05, 0.),
    mc_solver::Collision("RARM_LINK4", "CHEST_LINK0", 0.1, 0.05, 0.),
    mc_solver::Collision("RARM_LINK5", "CHEST_LINK0", 0.1, 0.05, 0.),
    mc_solver::Collision("RARM_LINK4", "CHEST_LINK1", 0.1, 0.05, 0.),
    mc_solver::Collision("RARM_LINK5", "CHEST_LINK1", 0.1, 0.05, 0.),
    mc_solver::Collision("LARM_LINK3", "CHEST_LINK0", 0.1, 0.05, 0.),
    mc_solver::Collision("LARM_LINK4", "CHEST_LINK0", 0.1, 0.05, 0.),
    mc_solver::Collision("LARM_LINK5", "CHEST_LINK0", 0.1, 0.05, 0.),
    mc_solver::Collision("LARM_LINK4", "CHEST_LINK1", 0.1, 0.05, 0.),
    mc_solver::Collision("LARM_LINK5", "CHEST_LINK1", 0.1, 0.05, 0.)
  });
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->solver.addTask(postureTask.get());
  qpsolver->setContacts({
    mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
    mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
    mc_rbdyn::Contact(robots(), "Butthock", "AllGround")
  });

  lh2Task.reset(new mc_tasks::RelativeEndEffectorTask("LARM_LINK6", qpsolver->robots, qpsolver->robots.robotIndex(), 0, 2.0, 100000.0));
  rh2Task.reset(new mc_tasks::RelativeEndEffectorTask("RARM_LINK6", qpsolver->robots, qpsolver->robots.robotIndex(), 0, 2.0, 100000.0));
  chestTask.reset(new mc_tasks::EndEffectorTask("CHEST_LINK1", qpsolver->robots, qpsolver->robots.robotIndex(), 1.0, 1e6));
  lh2Task->addToSolver(qpsolver->solver);
  rh2Task->addToSolver(qpsolver->solver);
  chestTask->addToSolver(qpsolver->solver);
  comTask.reset(new mc_tasks::CoMTask(qpsolver->robots, qpsolver->robots.robotIndex()));
  comTask->addToSolver(qpsolver->solver);
#ifdef MC_RTC_HAS_ROS
  if(mc_rtc::ROSBridge::get_node_handle())
  {
    tf_caster.reset(new tf2_ros::TransformBroadcaster());
  }
#endif
  std::cout << "MCBCISelfInteractController init done" << std::endl;
}

void MCBCISelfInteractController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  qpsolver->setContacts({
    mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
    mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
    mc_rbdyn::Contact(robots(), "Butthock", "AllGround")
  });
  lh2Task->resetTask(qpsolver->robots, qpsolver->robots.robotIndex());
  rh2Task->resetTask(qpsolver->robots, qpsolver->robots.robotIndex());
  chestTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex());
  comTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex());
}

bool MCBCISelfInteractController::run()
{
  bool ret = MCController::run();
  if(ret)
  {
    lh2Task->update();
    rh2Task->update();
    #ifdef MC_RTC_HAS_ROS
    if(tf_caster)
    {
      geometry_msgs::TransformStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "BODY";
      msg.header.seq = ++seq;
      msg.child_frame_id = "bci_left_hand_target";
      sva::PTransformd X = lh2Task->get_ef_pose();
      Eigen::Vector4d q = Eigen::Quaterniond(X.rotation()).inverse().coeffs();
      const Eigen::Vector3d & t = X.translation();

      msg.transform.translation.x = t.x();
      msg.transform.translation.y = t.y();
      msg.transform.translation.z = t.z();

      msg.transform.rotation.w = q.w();
      msg.transform.rotation.x = q.x();
      msg.transform.rotation.y = q.y();
      msg.transform.rotation.z = q.z();
      tf_caster->sendTransform(msg);
    }
    if(tf_caster)
    {
      geometry_msgs::TransformStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "BODY";
      msg.header.seq = ++seq;
      msg.child_frame_id = "bci_right_hand_target";
      sva::PTransformd X = rh2Task->get_ef_pose();
      Eigen::Vector4d q = Eigen::Quaterniond(X.rotation()).inverse().coeffs();
      const Eigen::Vector3d & t = X.translation();

      msg.transform.translation.x = t.x();
      msg.transform.translation.y = t.y();
      msg.transform.translation.z = t.z();

      msg.transform.rotation.w = q.w();
      msg.transform.rotation.x = q.x();
      msg.transform.rotation.y = q.y();
      msg.transform.rotation.z = q.z();
      tf_caster->sendTransform(msg);
    }
    #endif
  }
  return ret;
}

bool MCBCISelfInteractController::read_msg(std::string & msg)
{
  if(msg.size() == 0)
  {
    return false;
  }
  std::stringstream ss;
  ss << msg;
  std::string token;
  ss >> token;
  if(token == "GoToInitialPose")
  {
    bool lh;
    ss >> lh;
    if(lh)
    {
      Eigen::Matrix3d rot;
      rot << 0, 0, -1, 0, 1, 0, 1, 0, 0;
      Eigen::Vector3d t;
      t << 0.228722, 0.25107, 0.214931;
      lh2Task->set_ef_pose(sva::PTransformd(rot.inverse(), t));
    }
    else
    {
      Eigen::Matrix3d rot;
      rot << 0, 0, -1, 0, 1, 0, 1, 0, 0;
      Eigen::Vector3d t;
      t << 0.240164, -0.25107, 0.215508;
      rh2Task->set_ef_pose(sva::PTransformd(rot.inverse(), t));
    }
  }
  else if(token == "GoToWalkPose")
  {
    {
    Eigen::Matrix3d rot;
    rot << 0.824835, 0.241845, -0.511037, -0.330366, 0.939693, -0.0885213, 0.458809, 0.241845, 0.854988;
    Eigen::Vector3d t;
    t << -0.0060278, 0.379162, 0.0743646;
    lh2Task->set_ef_pose(sva::PTransformd(rot.inverse(), t));
    }
    {
    Eigen::Matrix3d rot;
    rot << 0.824835, -0.241845, -0.511037, 0.330366, 0.939693, 0.0885213, 0.458809, -0.241845, 0.854988;
    Eigen::Vector3d t;
    t << -0.0060278, -0.379162, 0.0743646;
    rh2Task->set_ef_pose(sva::PTransformd(rot.inverse(), t));
    }
  }
  else if(token == "GoToInitialRubberPose")
  {
    bool lh;
    ss >> lh;
    if(lh)
    {
      std::cout << "GoToInitialRubberPose - Left Hand (FIND IMPLEMENTATION)" << std::endl;
    }
    else
    {
      std::cout << "GoToInitialRubberPose - Right Hand (FIND IMPLEMENTATION)" << std::endl;
    }
  }
  else if(token == "SetEFPose")
  {
    std::string ef;
    ss >> ef;
    Eigen::Matrix3d ori;
    ss >> ori;
    Eigen::Vector3d t;
    ss >> t;
    if(ef == "lh2")
    {
      lh2Task->set_ef_pose(sva::PTransformd(ori.inverse(), t));
    }
    else if(ef == "rh2")
    {
      rh2Task->set_ef_pose(sva::PTransformd(ori.inverse(), t));
    }
    else
    {
      std::cerr << "Do not know how to set ef pose for: " << ef << std::endl;
    }
  }
  else if(token == "SetPanTilt")
  {
    double pan = 0; double tilt = 0;
    ss >> pan >> tilt;
    set_joint_pos("HEAD_JOINT0", pan);
    set_joint_pos("HEAD_JOINT1", tilt);
  }
  else
  {
    std::cerr << "BCISelfInteract controller cannot handle this token: " << token << std::endl;
    return false;
  }
  return true;
}

bool MCBCISelfInteractController::read_write_msg(std::string & msg, std::string & out)
{
  if(msg.size() == 0)
  {
    return false;
  }
  std::stringstream ss;
  ss << msg;
  std::string token;
  ss >> token;
  if(token == "Read")
  {
    sva::PTransformd out_X = sva::PTransformd::Identity();
    Eigen::Matrix4d out_m = Eigen::Matrix4d::Identity();
    std::string signal;
    ss >> signal;
    if(signal == "gaze")
    {
      out_X = robot().mbc().bodyPosW[robot().bodyIndexByName("HEAD_LINK1")];
    }
    else if(signal == "waist")
    {
      out_X = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];
    }
    else if(signal == "lh2")
    {
      out_X = lh2Task->get_ef_pose();
    }
    else if(signal == "rh2")
    {
      out_X = rh2Task->get_ef_pose();
    }
    else
    {
      std::cerr << "Do not know how to read signal: " << signal << std::endl;
    }
    Eigen::Matrix3d ori = Eigen::Quaterniond(out_X.rotation()).inverse().toRotationMatrix();
    for(int i = 0; i < 3; ++i)
    {
      for(int j = 0; j < 3; ++j)
      {
        out_m(i,j) = ori(i,j);
      }
      out_m(i, 3) = out_X.translation()(i);
    }
    std::stringstream ss_out;
    ss_out << out_m;
    out = ss_out.str();
  }
  else
  {
    std::cerr << "BCISelfInteract controller cannot handle this token: " << token << std::endl;
    return false;
  }
  return true;
}

}
