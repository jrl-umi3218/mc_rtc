#include <mc_control/mc_bci_self_interact_controller.h>

#include <mc_rtc/ros.h>
#ifdef MC_RTC_HAS_ROS
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#endif

namespace mc_control
{

MCBCISelfInteractController::MCBCISelfInteractController()
: MCController(),
  tf_caster(0), seq(0)
{
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(dynamicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->solver.addTask(postureTask.get());
  qpsolver->setContacts({
    mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
    mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
    mc_rbdyn::Contact(robots(), "Butthock", "AllGround")
  });

  std::cout << "MCBCISelfInteractController init done" << std::endl;
  efTask.reset(new mc_tasks::RelativeEndEffectorTask("RARM_LINK6", qpsolver->robots, qpsolver->robots.robotIndex(), 0, 10.0, 100.0));
  efTask->addToSolver(qpsolver->solver);
  comTask.reset(new mc_tasks::CoMTask(qpsolver->robots, qpsolver->robots.robotIndex()));
  comTask->addToSolver(qpsolver->solver);
#ifdef MC_RTC_HAS_ROS
  if(mc_rtc::ROSBridge::get_node_handle())
  {
    tf_caster.reset(new tf2_ros::TransformBroadcaster());
  }
#endif
}

void MCBCISelfInteractController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  qpsolver->setContacts({
    mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
    mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
    mc_rbdyn::Contact(robots(), "Butthock", "AllGround")
  });
  efTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex());
  comTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex());
}

bool MCBCISelfInteractController::run()
{
  bool ret = MCController::run();
  if(ret)
  {
    efTask->update();
    #ifdef MC_RTC_HAS_ROS
    if(tf_caster)
    {
      geometry_msgs::TransformStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "BODY";
      msg.header.seq = ++seq;
      msg.child_frame_id = "bci_hand_target";
      sva::PTransformd X = efTask->get_ef_pose();
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
  /* FIXME Obviously a test... */
  static bool done = false;
  if(!done)
  {
    std::cout << msg << std::endl;
    std::cout << "Emulate GoToInitialPose for RIGHT HAND" << std::endl;
    Eigen::Matrix3d rot;
    rot << -0.0143139, -0.290271, -0.956837, 0.0107325, 0.956836, -0.290431, 0.99984, -0.0144265, -0.0105807;
    Eigen::Vector3d t;
    t << 0.240164, -0.199974, 0.215508;
    efTask->set_ef_pose(sva::PTransformd(rot.inverse(), t));
    done = true;
  }
  else
  {
    /*FIXME Should be done in run() based on the task error */
    efTask->positionTaskSp->weight(10*efTask->positionTaskSp->weight());
    efTask->positionTaskSp->stiffness(2*efTask->positionTaskSp->stiffness());
    efTask->orientationTaskSp->weight(10*efTask->orientationTaskSp->weight());
    efTask->orientationTaskSp->stiffness(2*efTask->orientationTaskSp->stiffness());
  }
  return true;
}

}
