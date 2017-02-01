namespace mc_control
{

struct EgressPhaseExecution
{
public:
  /* Returns true if the phase is over */
  virtual bool run(MCEgressController & ctl) = 0;
};

struct EgressStartPhase : public EgressPhaseExecution
{
public:
  virtual bool run(MCEgressController &) override
  {
    return false;
    //return true;
  }
};

struct EgressMoveFootInsidePhase : public EgressPhaseExecution
{
public:
  EgressMoveFootInsidePhase()
  : done_setup_lift(false),
    done_lift(false),
    done_setup_rotate(false),
    done_rotate(false),
    done_setup_putdown(false),
    done_putdown(false),
    iterSincePutDown(0),
    iterForce(0)
  {
    LOG_INFO("EgressMoveFootInsidePhase")
  }
  virtual bool run(MCEgressController & ctl) override
  {
    if(!done_lift)
    {
      if(!done_setup_lift)
      {
        ctl.solver().setContacts({
          mc_rbdyn::Contact(ctl.robots(), "Butthock", "left_seat"),
          mc_rbdyn::Contact(ctl.robots(), "LFullSole", "exit_platform"),
          mc_rbdyn::Contact(ctl.robots(), "LeftThight", "left_seat"),
          mc_rbdyn::Contact(ctl.robots(), "RightGripper", "bar_wheel")
        });
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.robots(), ctl.robots().robotIndex()));
        ctl.solver().addTask(ctl.efTask);
        sva::PTransformd lift(Eigen::Vector3d(-0.125, 0, 0.15)); /*XXX Hard-coded value */
        ctl.efTask->set_ef_pose(ctl.efTask->get_ef_pose()*lift);
        iterSincePutDown = 0;
        done_setup_lift = true;
      }
      else
      {
        iterSincePutDown++;
        double error = ctl.efTask->positionTask->eval().norm();
        if(error < 0.05 || iterSincePutDown > 15*500)
        {
          ctl.solver().removeTask(ctl.efTask);
          ctl.postureTask->posture(ctl.robot().mbc().q);
          done_lift = true;
          LOG_INFO("Finished lift phase")
        }
      }
    }
    else if(!done_rotate)
    {
      if(!done_setup_rotate)
      {
        ctl.oriTask.reset(new mc_tasks::OrientationTask("RLEG_LINK5", ctl.robots(), 0));
        ctl.solver().addTask(ctl.oriTask);
        Eigen::Matrix3d change = sva::RotZ(40*M_PI/180); /*XXX Hard-coded value */
        ctl.oriTask->orientation(ctl.oriTask->orientation()*change);
        done_setup_rotate = true;
      }
      else
      {
        double error = ctl.oriTask->eval().norm();
        if(error < 0.01)
        {
          ctl.solver().removeTask(ctl.oriTask);
          ctl.postureTask->posture(ctl.robot().mbc().q);
          done_rotate = true;
          LOG_INFO("Finished rotate foot")
        }
      }
    }
    else if(!done_putdown)
    {
      if(!done_setup_putdown)
      {
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.robots(), 0, 0.25));
        ctl.solver().addTask(ctl.efTask);
        sva::PTransformd down(Eigen::Vector3d(0.01,0.01, -0.3)); /*XXX Hard-coded*/
        unsigned int bIdx = ctl.robot().bodyIndexByName("LLEG_LINK5");
        sva::PTransformd bpw = ctl.robot().mbc().bodyPosW[bIdx];
        ctl.efTask->set_ef_pose(sva::PTransformd(sva::RotZ(-M_PI/8)*bpw.rotation(), (ctl.efTask->get_ef_pose()*down).translation()));
        iterSincePutDown = 0;
        iterForce = 0;
        done_setup_putdown = true;
      }
      else
      {
        iterSincePutDown++;
        if(ctl.robot().forceSensor("RightFootForceSensor").force()[0] > 100)
        {
          iterForce++;
        }
        else
        {
          iterForce = 0;
        }
        if(iterForce > 20 || iterSincePutDown > 15*500)
        {
          ctl.solver().removeTask(ctl.efTask);
          ctl.postureTask->posture(ctl.robot().mbc().q);
          done_putdown = true;
          LOG_INFO("Contact found, next step")
          ctl.solver().setContacts({
            mc_rbdyn::Contact(ctl.robots(), "Butthock", "left_seat"),
            mc_rbdyn::Contact(ctl.robots(), "LFullSole", "exit_platform"),
            mc_rbdyn::Contact(ctl.robots(), "RFullSole", "left_floor"),
            mc_rbdyn::Contact(ctl.robots(), "RightGripper", "bar_wheel")
          });
          //return true;
        }
      }
    }
    return false;
  }
private:
  bool done_setup_lift;
  bool done_lift;
  bool done_setup_rotate;
  bool done_rotate;
  bool done_setup_putdown;
  bool done_putdown;
  unsigned int iterSincePutDown;
  unsigned int iterForce;
};

struct EgressRemoveHandPhase : public EgressPhaseExecution
{
public:
  EgressRemoveHandPhase()
  : done_setup_normal_move(false),
    done_normal_move(false),
    done_setup_move_right(false),
    done_move_right(false),
    done_setup_go_to_posture(false),
    done_go_to_posture(false),
    iterSinceMoving(0)
  {
  }
  virtual bool run(MCEgressController & ctl) override
  {
    if(!done_normal_move)
    {
      if(!done_setup_normal_move)
      {
        ctl.solver().setContacts({
          mc_rbdyn::Contact(ctl.robots(), "Butthock", "left_seat"),
          mc_rbdyn::Contact(ctl.robots(), "LFullSole", "exit_platform"),
          mc_rbdyn::Contact(ctl.robots(), "RFullSole", "left_floor"),
          mc_rbdyn::Contact(ctl.robots(), "LowerBack", "left_back"),
        });
        unsigned int bIdx = ctl.robot().bodyIndexByName("RARM_LINK6");
        sva::PTransformd bpw = ctl.robot().mbc().bodyPosW[bIdx];
        initPos = bpw.translation();
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6", ctl.robots(), 0));
        ctl.solver().addTask(ctl.efTask);
        const auto & gripperSurface = ctl.robot().surface("RightGripper");
        sva::PTransformd X_0_s = gripperSurface.X_0_s(ctl.robot(), ctl.robot().mbc());
        Eigen::Vector3d normal = X_0_s.rotation().row(2);
        targetSpeed = 0.02*normal; /*XXX Hard-coded*/
        done_setup_normal_move = true;
      }
      else
      {
        unsigned int bIdx = ctl.robot().bodyIndexByName("RARM_LINK6");
        sva::PTransformd bpw = ctl.robot().mbc().bodyPosW[bIdx];
        double d = (bpw.translation() - initPos).norm();
        if(d > 0.13) /*XXX Hard-coded*/
        {
          ctl.postureTask->posture(ctl.robot().mbc().q);
          ctl.solver().removeTask(ctl.efTask);
          LOG_INFO("Hand removed from wheel")
          done_normal_move = true;
        }
        else
        {
          ctl.efTask->set_ef_pose(sva::PTransformd(bpw.rotation(), bpw.translation() + targetSpeed));
        }
      }
    }
    else if(!done_move_right)
    {
      if(!done_setup_move_right)
      {
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6", ctl.robots(), 0, 0.5));
        ctl.solver().addTask(ctl.efTask);
        sva::PTransformd move(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.1, -0.2, 0.1));
        ctl.efTask->set_ef_pose(ctl.efTask->get_ef_pose()*move);
        iterSinceMoving = 0;
        done_setup_move_right = true;
      }
      else
      {
        double error = ctl.efTask->positionTask->eval().norm();
        iterSinceMoving++;
        if(error < 0.05 || iterSinceMoving > 15*500)
        {
          ctl.postureTask->posture(ctl.robot().mbc().q);
          ctl.solver().removeTask(ctl.efTask);
          LOG_INFO("Hand moved sideway")
          done_move_right = true;
        }
      }
    }
    else if(!done_go_to_posture)
    {
      if(!done_setup_go_to_posture)
      {
        auto rarm0 = ctl.robot().jointIndexByName("RARM_JOINT0");
        auto p = ctl.postureTask->posture();
        p[rarm0][0]   = 0*M_PI/180;
        p[rarm0+1][0] = -80*M_PI/180;
        p[rarm0+2][0] = -80*M_PI/180;
        p[rarm0+3][0] = -130*M_PI/180;
        p[rarm0+4][0] = 0*M_PI/180;
        p[rarm0+5][0] = 0*M_PI/180;
        p[rarm0+6][0] = 0*M_PI/180;
        ctl.postureTask->posture(p);
        ctl.grippers["r_gripper"]->setTargetQ({0});
        done_setup_go_to_posture = true;
      }
      else
      {
        double error = ctl.postureTask->eval().norm();
        if(error < 0.01)
        {
          ctl.postureTask->posture(ctl.robot().mbc().q);
          ctl.solver().setContacts({
            mc_rbdyn::Contact(ctl.robots(), "Butthock", "left_seat"),
            mc_rbdyn::Contact(ctl.robots(), "LFullSole", "exit_platform"),
            mc_rbdyn::Contact(ctl.robots(), "RFullSole", "left_floor"),
          });
          done_go_to_posture = true;
          LOG_INFO("Arm reached a safe posture")
          //return true;
        }
      }
    }
    return false;
  }
private:
  bool done_setup_normal_move;
  bool done_normal_move;
  bool done_setup_move_right;
  bool done_move_right;
  bool done_setup_go_to_posture;
  bool done_go_to_posture;
  Eigen::Vector3d initPos;
  Eigen::Vector3d targetSpeed;
  unsigned int iterSinceMoving;
};

struct EgressRotateBodyPhase : public EgressPhaseExecution
{
public:
  EgressRotateBodyPhase()
  : done_setup_rotate_body(false),
    done_rotate_body(false),
    timeoutIter(0)
  {
  }
  virtual bool run(MCEgressController & ctl) override
  {
    if(!done_rotate_body)
    {
      if(!done_setup_rotate_body)
      {
        ctl.solver().setContacts({
          mc_rbdyn::Contact(ctl.robots(), "LFullSole", "exit_platform"),
          mc_rbdyn::Contact(ctl.robots(), "RFullSole", "left_floor"),
        });
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("BODY", ctl.robots(), ctl.robots().robotIndex(), 0.5));
        ctl.solver().addTask(ctl.efTask);
        unsigned int bIdx = ctl.robot().bodyIndexByName("LLEG_LINK5");
        sva::PTransformd bpw = ctl.robot().mbc().bodyPosW[bIdx];
        ctl.efTask->set_ef_pose(sva::PTransformd(bpw.rotation(), ctl.efTask->get_ef_pose().translation()));
        done_setup_rotate_body = true;
        timeoutIter = 0;
      }
      else
      {
        timeoutIter++;
        double error = ctl.efTask->orientationTask->eval().norm();
        if(error < 0.01 || timeoutIter > 15*500)
        {
          ctl.solver().setContacts({
            mc_rbdyn::Contact(ctl.robots(), "Butthock", "left_seat"),
            mc_rbdyn::Contact(ctl.robots(), "LFullSole", "exit_platform"),
            mc_rbdyn::Contact(ctl.robots(), "RFullSole", "left_floor"),
          });
          ctl.postureTask->posture(ctl.robot().mbc().q);
          ctl.solver().removeTask(ctl.efTask);
          done_rotate_body = true;
          LOG_INFO("Finished rotating the body")
          //return true;
        }
      }
    }
    return false;
  }
private:
  bool done_setup_rotate_body;
  bool done_rotate_body;
  unsigned int timeoutIter;
};

struct EgressMoveFootOutPhase : public EgressPhaseExecution
{
  EgressMoveFootOutPhase()
  : done_setup_lift(false),
    done_lift(false),
    done_setup_reorient(false),
    done_reorient(false),
    done_setup_rotate(false),
    done_rotate(false),
    done_setup_putdown(false),
    done_putdown(false),
    ankle_i(0), ankle_reorient_target(0),
    timeoutIter(0)
  {
  }


  virtual bool run(MCEgressController & ctl) override
  {
    if(!done_lift)
    {
      if(!done_setup_lift)
      {
        ctl.solver().setContacts({
          mc_rbdyn::Contact(ctl.robots(), "Butthock", "left_seat"),
          mc_rbdyn::Contact(ctl.robots(), "LFullSole", "exit_platform"),
        });
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.robots(), ctl.robots().robotIndex()));
        ctl.solver().addTask(ctl.efTask);
        sva::PTransformd lift(Eigen::Vector3d(0, 0, 0.05)); /*XXX Hard-coded value */
        ctl.efTask->set_ef_pose(ctl.efTask->get_ef_pose()*lift);
        done_setup_lift = true;
      }
      else
      {
        double error = ctl.efTask->positionTask->eval().norm();
        if(error < 0.01)
        {
          ctl.solver().removeTask(ctl.efTask);
          ctl.postureTask->posture(ctl.robot().mbc().q);
          done_lift = true;
          LOG_INFO("Finished lift phase")
        }
      }
    }
    else if(!done_reorient)
    {
      if(!done_setup_reorient)
      {
        ankle_i = ctl.robot().jointIndexByName("RLEG_JOINT4");
        auto p = ctl.postureTask->posture();
        ankle_reorient_target = -1.0; /*XXX Hard-coded value */
        p[ankle_i][0] = ankle_reorient_target;
        ctl.postureTask->posture(p);
        done_setup_reorient = true;
      }
      else
      {
        double error = std::abs(ctl.robot().mbc().q[ankle_i][0] - ankle_reorient_target);
        if(error < 0.01)
        {
          ctl.postureTask->posture(ctl.robot().mbc().q);
          done_reorient = true;
          LOG_INFO("Finished changing ankle orientation")
        }
      }
    }
    else if(!done_rotate)
    {
      if(!done_setup_rotate)
      {
        ctl.oriTask.reset(new mc_tasks::OrientationTask("RLEG_LINK5", ctl.robots(), 0, 0.5));
        ctl.solver().addTask(ctl.oriTask);
        Eigen::Matrix3d change = sva::RotZ(40*M_PI/180); /*XXX Hard-coded value */
        ctl.oriTask->orientation(ctl.oriTask->orientation()*change);
        timeoutIter = 0;
        done_setup_rotate = true;
      }
      else
      {
        timeoutIter++;
        double error = ctl.oriTask->eval().norm();
        if(error < 0.01 || timeoutIter > 15*500)
        {
          ctl.solver().removeTask(ctl.oriTask);
          ctl.postureTask->posture(ctl.robot().mbc().q);
          done_rotate = true;
          LOG_INFO("Finished rotate foot")
        }
      }
    }
    else if(!done_putdown)
    {
      if(!done_setup_putdown)
      {
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.robots(), 0, 0.5));
        ctl.solver().addTask(ctl.efTask);
        sva::PTransformd lmod(Eigen::Vector3d(0.2, -0.1, -0.1)); /*XXX Hard-coded value */
        unsigned int bIdx = ctl.robot().bodyIndexByName("LLEG_LINK5");
        sva::PTransformd bpw = ctl.robot().mbc().bodyPosW[bIdx];
        sva::PTransformd targetEfPose = bpw*lmod;
        ctl.efTask->set_ef_pose(targetEfPose);
        timeoutIter = 0;
        done_setup_putdown = true;
      }
      else
      {
        timeoutIter++;
        double error = ctl.efTask->positionTask->eval().norm();
        if(ctl.robot().forceSensor("RightFootForceSensor").force()[0] > 50 || error < 0.05 || timeoutIter > 15*500)
        {
          ctl.solver().removeTask(ctl.efTask);
          ctl.postureTask->posture(ctl.robot().mbc().q);
          ctl.solver().setContacts({
            mc_rbdyn::Contact(ctl.robots(), "Butthock", "left_seat"),
            mc_rbdyn::Contact(ctl.robots(), "LFullSole", "exit_platform"),
            mc_rbdyn::Contact(ctl.robots(), "RFullSole", "exit_platform")
          });
          done_putdown = true;
          LOG_INFO("Contact found, next step")
          //return true;
        }
      }
    }
    return false;
  }
private:
  bool done_setup_lift;
  bool done_lift;
  bool done_setup_reorient;
  bool done_reorient;
  bool done_setup_rotate;
  bool done_rotate;
  bool done_setup_putdown;
  bool done_putdown;
  unsigned int ankle_i;
  double ankle_reorient_target;
  unsigned int timeoutIter;
};

struct EgressCorrectLeftFootPhase : public EgressPhaseExecution
{
public:
  EgressCorrectLeftFootPhase()
  : done_setup_lift(false),
    done_lift(false),
    done_setup_rotate(false),
    done_rotate(false),
    done_setup_putdown(false),
    done_putdown(false),
    iterForce(0), timeoutIter(0)
  {
  }
  virtual bool run(MCEgressController & ctl) override
  {
    if(!done_lift)
    {
      if(!done_setup_lift)
      {
        ctl.solver().setContacts({
          mc_rbdyn::Contact(ctl.robots(), "Butthock", "left_seat"),
          mc_rbdyn::Contact(ctl.robots(), "RFullSole", "left_floor"),
          mc_rbdyn::Contact(ctl.robots(), "RightThight", "left_seat"),
        });
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("LLEG_LINK5", ctl.robots(), ctl.robots().robotIndex()));
        ctl.solver().addTask(ctl.efTask);
        sva::PTransformd lift(Eigen::Vector3d(0, 0, 0.1)); /*XXX Hard-coded value */
        ctl.efTask->set_ef_pose(ctl.efTask->get_ef_pose()*lift);
        timeoutIter = 0;
        done_setup_lift = true;
      }
      else
      {
        timeoutIter++;
        double error = ctl.efTask->positionTask->eval().norm();
        if(error < 0.05 || timeoutIter > 15*500)
        {
          ctl.solver().removeTask(ctl.efTask);
          ctl.postureTask->posture(ctl.robot().mbc().q);
          done_lift = true;
          LOG_INFO("Finished lift phase")
        }
      }
    }
    else if(!done_rotate)
    {
      if(!done_setup_rotate)
      {
        ctl.oriTask.reset(new mc_tasks::OrientationTask("LLEG_LINK5", ctl.robots(), 0, 1.0));
        ctl.solver().addTask(ctl.oriTask);
        Eigen::Matrix3d change = sva::RotZ(30*M_PI/180); /*XXX Hard-coded value */
        ctl.oriTask->orientation(ctl.oriTask->orientation()*change);
        done_setup_rotate = true;
        timeoutIter = 0;
      }
      else
      {
        timeoutIter++;
        double error = ctl.oriTask->eval().norm();
        if(error < 0.01 || timeoutIter > 15*500)
        {
          ctl.solver().removeTask(ctl.oriTask);
          ctl.postureTask->posture(ctl.robot().mbc().q);
          done_rotate = true;
          LOG_INFO("Finished rotate foot")
        }
      }
    }
    else if(!done_putdown)
    {
      if(!done_setup_putdown)
      {
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("LLEG_LINK5", ctl.robots(), 0, 0.25));
        ctl.solver().addTask(ctl.efTask);
        sva::PTransformd down(Eigen::Vector3d(0.0,0.0, -0.2)); /*XXX Hard-coded*/
        ctl.efTask->set_ef_pose(ctl.efTask->get_ef_pose()*down);
        timeoutIter = 0;
        iterForce = 0;
        done_setup_putdown = true;
      }
      else
      {
        timeoutIter++;
        if(ctl.robot().forceSensor("RightFootForceSensor").force()[0] > 100)
        {
          iterForce++;
        }
        else
        {
          iterForce = 0;
        }
        if(iterForce > 20 || timeoutIter > 15*500)
        {
          ctl.solver().removeTask(ctl.efTask);
          ctl.postureTask->posture(ctl.robot().mbc().q);
          done_putdown = true;
          LOG_INFO("Contact found, next step")
          ctl.solver().setContacts({
            mc_rbdyn::Contact(ctl.robots(), "Butthock", "left_seat"),
            mc_rbdyn::Contact(ctl.robots(), "LFullSole", "exit_platform"),
            mc_rbdyn::Contact(ctl.robots(), "RFullSole", "left_floor"),
          });
          //return true;
        }
      }
    }
    return false;
  }
private:
  bool done_setup_lift;
  bool done_lift;
  bool done_setup_rotate;
  bool done_rotate;
  bool done_setup_putdown;
  bool done_putdown;
  unsigned int iterForce;
  unsigned int timeoutIter;
};

struct EgressStandupPhase : public EgressPhaseExecution
{
  EgressStandupPhase()
  : done_setup_standup(false),
    done_standup(false)
  {
  }

  virtual bool run(MCEgressController & ctl) override
  {
    if(!done_standup)
    {
      if(!done_setup_standup)
      {
        ctl.solver().setContacts({
          mc_rbdyn::Contact(ctl.robots(), "LFullSole", "exit_platform"),
          mc_rbdyn::Contact(ctl.robots(), "RFullSole", "exit_platform"),
        });
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("BODY", ctl.robots(), ctl.robots().robotIndex(), 1.0));
        ctl.solver().addTask(ctl.efTask);
        unsigned int lbIdx = ctl.robot().bodyIndexByName("LLEG_LINK5");
        sva::PTransformd lbpw = ctl.robot().mbc().bodyPosW[lbIdx];
        unsigned int rbIdx = ctl.robot().bodyIndexByName("RLEG_LINK5");
        sva::PTransformd rbpw = ctl.robot().mbc().bodyPosW[rbIdx];
        Eigen::Vector3d bodyPos = (lbpw.translation() + rbpw.translation())/2;
        bodyPos(2) = bodyPos(2) + 0.76 - 0.15;
        ctl.efTask->set_ef_pose(sva::PTransformd(ctl.efTask->get_ef_pose().rotation(), bodyPos));
        done_setup_standup = true;
      }
      else
      {
        double error = ctl.efTask->positionTask->eval().norm();
        if(error < 0.05)
        {
          ctl.postureTask->posture(ctl.robot().mbc().q);
          ctl.solver().removeTask(ctl.efTask);
          done_standup = true;
          LOG_INFO("Done standup")
          //return true;
        }
      }
    }
    return false;
  }
private:
  bool done_setup_standup;
  bool done_standup;
};

}
