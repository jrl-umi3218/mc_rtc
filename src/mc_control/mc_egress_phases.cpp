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
  virtual bool run(MCEgressController & ctl) override
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
    done_setup_reorient(false),
    done_reorient(false),
    done_setup_rotate(false),
    done_rotate(false),
    done_setup_flatten(false),
    done_flatten(false),
    done_setup_putdown(false),
    done_putdown(false)
  {
    std::cout << "EgressMoveFootInsidePhase" << std::endl;
  }
  virtual bool run(MCEgressController & ctl) override
  {
    if(!done_lift)
    {
      if(!done_setup_lift)
      {
        ctl.qpsolver->setContacts({
          mc_rbdyn::Contact(ctl.robot().surfaces.at("Butthock"), ctl.env().surfaces.at("left_seat")),
          mc_rbdyn::Contact(ctl.robot().surfaces.at("LFullSole"), ctl.env().surfaces.at("exit_platform")),
          mc_rbdyn::Contact(ctl.robot().surfaces.at("LeftThight"), ctl.env().surfaces.at("left_seat")),
          mc_rbdyn::Contact(ctl.robot().surfaces.at("RightGripper"), ctl.env().surfaces.at("bar_wheel"))
        });
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.qpsolver->robots, ctl.qpsolver->robots.robotIndex));
        ctl.efTask->addToSolver(ctl.qpsolver->solver);
        Eigen::Vector3d lift(-0.125, 0, 0.15); /*XXX Hard-coded value */
        lift = ctl.efTask->get_ef_pose().rotation()*lift + ctl.efTask->get_ef_pose().translation();
        //ctl.efTask->set_ef_pose(sva::PTransformd(Eigen::Matrix3d::Identity(), lift)*ctl.efTask->get_ef_pose());
        ctl.efTask->set_ef_pose(sva::PTransformd(ctl.efTask->get_ef_pose().rotation(), lift));
        iterSincePutDown = 0;
        done_setup_lift = true;
      }
      else
      {
        iterSincePutDown++;
        double error = ctl.efTask->positionTask->eval().norm();
        if(error < 0.05 || iterSincePutDown > 15*500)
        {
          ctl.efTask->removeFromSolver(ctl.qpsolver->solver);
          ctl.postureTask->posture(ctl.robot().mbc->q);
          done_lift = true;
          std::cout << "Finished lift phase" << std::endl;
        }
      }
    }
    else if(!done_reorient)
    {
      if(!done_setup_reorient)
      {
        ankle_i = ctl.robot().jointIndexByName("RLEG_JOINT4");
        auto p = ctl.postureTask->posture();
        ankle_reorient_target = p[ankle_i][0]; /*XXX Hard-coded value */
        p[ankle_i][0] = ankle_reorient_target;
        ctl.postureTask->posture(p);
        done_setup_reorient = true;
      }
      else
      {
        double error = std::abs(ctl.robot().mbc->q[ankle_i][0] - ankle_reorient_target);
        if(error < 0.01)
        {
          ctl.postureTask->posture(ctl.robot().mbc->q);
          done_reorient = true;
          std::cout << "Finished changing ankle orientation" << std::endl;
        }
      }
    }
    else if(!done_rotate)
    {
      if(!done_setup_rotate)
      {
        ctl.oriTask.reset(new mc_tasks::OrientationTask("RLEG_LINK5", ctl.robots(), 0));
        ctl.oriTask->addToSolver(ctl.qpsolver->solver);
        Eigen::Matrix3d change = sva::RotZ(40*M_PI/180); /*XXX Hard-coded value */
        ctl.oriTask->set_ef_ori(ctl.oriTask->get_ef_ori()*change);
        done_setup_rotate = true;
      }
      else
      {
        double error = ctl.oriTask->orientationTask->eval().norm();
        if(error < 0.01)
        {
          ctl.oriTask->removeFromSolver(ctl.qpsolver->solver);
          ctl.postureTask->posture(ctl.robot().mbc->q);
          done_rotate = true;
          std::cout << "Finished rotate foot" << std::endl;
        }
      }
    }
    else if(!done_flatten)
    {
      if(!done_setup_flatten)
      {
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.qpsolver->robots, 0, 0.5));
        ctl.efTask->addToSolver(ctl.qpsolver->solver);
        unsigned int bIdx = ctl.robot().bodyIndexByName("LLEG_LINK5");
        sva::PTransformd bpw = ctl.robot().mbc->bodyPosW[bIdx];
        ctl.efTask->set_ef_pose(sva::PTransformd(bpw.rotation(), ctl.efTask->get_ef_pose().translation()));
        iterSincePutDown = 0;
        done_setup_flatten = true;
      }
      else
      {
        iterSincePutDown++;
        double error = ctl.efTask->orientationTask->eval().norm();
        if(error < 0.1 || iterSincePutDown > 15*500)
        {
          ctl.efTask->removeFromSolver(ctl.qpsolver->solver);
          ctl.postureTask->posture(ctl.robot().mbc->q);
          done_flatten = true;
          std::cout << "Finished flatten foot" << std::endl;
        }
      }
    }
    else if(!done_putdown)
    {
      if(!done_setup_putdown)
      {
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.qpsolver->robots, 0, 0.05));
        ctl.efTask->addToSolver(ctl.qpsolver->solver);
        Eigen::Vector3d down(0.01, 0.01, -0.2); /*XXX Hard-coded value */
        down = down + ctl.efTask->get_ef_pose().translation();
        ctl.efTask->set_ef_pose(sva::PTransformd(ctl.efTask->get_ef_pose().rotation(), down));
        iterSincePutDown = 0;
        iterForce = 0;
        done_setup_putdown = true;
      }
      else
      {
        iterSincePutDown++;
        double error = ctl.efTask->positionTask->eval().norm();
        if(ctl.wrenches[0].first[0] > 100)
        {
          iterForce++;
        }
        else
        {
          iterForce = 0;
        }
        if(iterForce > 20 || iterSincePutDown > 15*500)
        {
          ctl.efTask->removeFromSolver(ctl.qpsolver->solver);
          ctl.postureTask->posture(ctl.robot().mbc->q);
          done_putdown = true;
          std::cout << "Contact found, next step" << std::endl;
          ctl.qpsolver->setContacts({
            mc_rbdyn::Contact(ctl.robot().surfaces.at("Butthock"), ctl.env().surfaces.at("left_seat")),
            mc_rbdyn::Contact(ctl.robot().surfaces.at("LFullSole"), ctl.env().surfaces.at("exit_platform")),
            mc_rbdyn::Contact(ctl.robot().surfaces.at("RFullSole"), ctl.env().surfaces.at("left_floor")),
            mc_rbdyn::Contact(ctl.robot().surfaces.at("RightGripper"), ctl.env().surfaces.at("bar_wheel"))
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
  bool done_setup_reorient;
  bool done_reorient;
  bool done_setup_rotate;
  bool done_rotate;
  bool done_setup_flatten;
  bool done_flatten;
  bool done_setup_putdown;
  bool done_putdown;
  int ankle_i;
  double ankle_reorient_target;
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
    done_go_to_posture(false)
  {
  }
  virtual bool run(MCEgressController & ctl) override
  {
    if(!done_normal_move)
    {
      if(!done_setup_normal_move)
      {
        ctl.qpsolver->setContacts({
          mc_rbdyn::Contact(ctl.robot().surfaces.at("Butthock"), ctl.env().surfaces.at("left_seat")),
          mc_rbdyn::Contact(ctl.robot().surfaces.at("LFullSole"), ctl.env().surfaces.at("exit_platform")),
          mc_rbdyn::Contact(ctl.robot().surfaces.at("RFullSole"), ctl.env().surfaces.at("left_floor")),
          mc_rbdyn::Contact(ctl.robot().surfaces.at("LowerBack"), ctl.env().surfaces.at("left_back")),
        });
        unsigned int bIdx = ctl.robot().bodyIndexByName("RARM_LINK6");
        sva::PTransformd bpw = ctl.robot().mbc->bodyPosW[bIdx];
        initPos = bpw.translation();
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6", ctl.qpsolver->robots, 0));
        ctl.efTask->addToSolver(ctl.qpsolver->solver);
        auto gripperSurface = ctl.robot().surfaces.at("RightGripper");
        sva::PTransformd X_0_s = gripperSurface->X_0_s(ctl.robot(), *(ctl.robot().mbc));
        Eigen::Vector3d normal = X_0_s.rotation().row(2);
        targetSpeed = 0.02*normal; /*XXX Hard-coded*/
        done_setup_normal_move = true;
      }
      else
      {
        unsigned int bIdx = ctl.robot().bodyIndexByName("RARM_LINK6");
        sva::PTransformd bpw = ctl.robot().mbc->bodyPosW[bIdx];
        double d = (bpw.translation() - initPos).norm();
        if(d > 0.14) /*XXX Hard-coded*/
        {
          ctl.postureTask->posture(ctl.robot().mbc->q);
          ctl.efTask->removeFromSolver(ctl.qpsolver->solver);
          std::cout << "Hand removed from wheel" << std::endl;
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
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6", ctl.qpsolver->robots, 0, 0.5));
        ctl.efTask->addToSolver(ctl.qpsolver->solver);
        Eigen::Vector3d move(0., -0.3, 0.); /*XXX Hard-coded*/
        move = ctl.efTask->get_ef_pose().rotation()*move + ctl.efTask->get_ef_pose().translation();
        ctl.efTask->set_ef_pose(sva::PTransformd(ctl.efTask->get_ef_pose().rotation(), move));
        iterSinceMoving = 0;
        done_setup_move_right = true;
      }
      else
      {
        double error = ctl.efTask->positionTask->eval().norm();
        iterSinceMoving++;
        if(error < 0.1 || iterSinceMoving > 15*500)
        {
          ctl.postureTask->posture(ctl.robot().mbc->q);
          ctl.efTask->removeFromSolver(ctl.qpsolver->solver);
          std::cout << "Hand moved sideway" << std::endl;
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
        ctl.rgripper->setTargetQ(0);
        done_setup_go_to_posture = true;
      }
      else
      {
        double error = ctl.postureTask->eval().norm();
        if(error < 0.01)
        {
          ctl.postureTask->posture(ctl.robot().mbc->q);
          ctl.qpsolver->setContacts({
            mc_rbdyn::Contact(ctl.robot().surfaces.at("Butthock"), ctl.env().surfaces.at("left_seat")),
            mc_rbdyn::Contact(ctl.robot().surfaces.at("LFullSole"), ctl.env().surfaces.at("exit_platform")),
            mc_rbdyn::Contact(ctl.robot().surfaces.at("RFullSole"), ctl.env().surfaces.at("left_floor")),
          });
          done_go_to_posture = true;
          std::cout << "Arm reached a safe posture" << std::endl;
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
    done_rotate_body(false)
  {
  }
  virtual bool run(MCEgressController & ctl) override
  {
    if(!done_rotate_body)
    {
      if(!done_setup_rotate_body)
      {
        ctl.qpsolver->setContacts({
          mc_rbdyn::Contact(ctl.robot().surfaces.at("LFullSole"), ctl.env().surfaces.at("exit_platform")),
          mc_rbdyn::Contact(ctl.robot().surfaces.at("RFullSole"), ctl.env().surfaces.at("left_floor")),
        });
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("BODY", ctl.qpsolver->robots, ctl.qpsolver->robots.robotIndex));
        ctl.efTask->addToSolver(ctl.qpsolver->solver);
        Eigen::Matrix3d changeOri = sva::RotZ(M_PI/6); /*XXX Hard-coded*/
        ctl.efTask->set_ef_pose(sva::PTransformd(changeOri, Eigen::Vector3d::Zero())*ctl.efTask->get_ef_pose());
        done_setup_rotate_body = true;
      }
      else
      {
        double error = ctl.efTask->orientationTask->eval().norm();
        if(error < 0.01)
        {
          ctl.qpsolver->setContacts({
            mc_rbdyn::Contact(ctl.robot().surfaces.at("Butthock"), ctl.env().surfaces.at("left_seat")),
            mc_rbdyn::Contact(ctl.robot().surfaces.at("LFullSole"), ctl.env().surfaces.at("exit_platform")),
            mc_rbdyn::Contact(ctl.robot().surfaces.at("RFullSole"), ctl.env().surfaces.at("left_floor")),
          });
          ctl.efTask->removeFromSolver(ctl.qpsolver->solver);
          done_rotate_body = true;
          std::cout << "Finished rotating the body" << std::endl;
          //return true;
        }
      }
    }
    return false;
  }
private:
  bool done_setup_rotate_body;
  bool done_rotate_body;
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
    done_setup_flatten(false),
    done_flatten(false),
    done_setup_putdown(false),
    done_putdown(false)
  {
  }


  virtual bool run(MCEgressController & ctl) override
  {
    if(!done_lift)
    {
      if(!done_setup_lift)
      {
        ctl.qpsolver->setContacts({
          mc_rbdyn::Contact(ctl.robot().surfaces.at("Butthock"), ctl.env().surfaces.at("left_seat")),
          mc_rbdyn::Contact(ctl.robot().surfaces.at("LFullSole"), ctl.env().surfaces.at("exit_platform")),
        });
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.qpsolver->robots, ctl.qpsolver->robots.robotIndex));
        ctl.efTask->addToSolver(ctl.qpsolver->solver);
        Eigen::Vector3d lift(0, 0, 0.1); /*XXX Hard-coded value */
        ctl.efTask->set_ef_pose(sva::PTransformd(Eigen::Matrix3d::Identity(), lift)*ctl.efTask->get_ef_pose());
        done_setup_lift = true;
      }
      else
      {
        double error = ctl.efTask->positionTask->eval().norm();
        if(error < 0.01)
        {
          ctl.efTask->removeFromSolver(ctl.qpsolver->solver);
          ctl.postureTask->posture(ctl.robot().mbc->q);
          done_lift = true;
          std::cout << "Finished lift phase" << std::endl;
        }
      }
    }
    else if(!done_reorient)
    {
      if(!done_setup_reorient)
      {
        ankle_i = ctl.robot().jointIndexByName("RLEG_JOINT4");
        ankle_reorient_target = -1.25; /*XXX Hard-coded value */
        auto p = ctl.postureTask->posture();
        p[ankle_i][0] = ankle_reorient_target;
        ctl.postureTask->posture(p);
        done_setup_reorient = true;
      }
      else
      {
        double error = std::abs(ctl.robot().mbc->q[ankle_i][0] - ankle_reorient_target);
        if(error < 0.01)
        {
          ctl.postureTask->posture(ctl.robot().mbc->q);
          done_reorient = true;
          std::cout << "Finished changing ankle orientation" << std::endl;
        }
      }
    }
    else if(!done_rotate)
    {
      if(!done_setup_rotate)
      {
        ctl.oriTask.reset(new mc_tasks::OrientationTask("RLEG_LINK5", ctl.robots(), 0, 0.5));
        ctl.oriTask->addToSolver(ctl.qpsolver->solver);
        Eigen::Matrix3d change = sva::RotZ(35*M_PI/180); /*XXX Hard-coded value */
        ctl.oriTask->set_ef_ori(ctl.oriTask->get_ef_ori()*change);
        done_setup_rotate = true;
      }
      else
      {
        double error = ctl.oriTask->orientationTask->eval().norm();
        if(error < 0.01)
        {
          ctl.oriTask->removeFromSolver(ctl.qpsolver->solver);
          ctl.postureTask->posture(ctl.robot().mbc->q);
          done_rotate = true;
          std::cout << "Finished rotate foot" << std::endl;
        }
      }
    }
    else if(!done_flatten)
    {
      if(!done_setup_flatten)
      {
        ankle_i = ctl.robot().jointIndexByName("RLEG_JOINT4");
        auto lankle_i = ctl.robot().jointIndexByName("LLEG_JOINT4");
        auto roll_i = ctl.robot().jointIndexByName("RLEG_JOINT5");
        auto lroll_i = ctl.robot().jointIndexByName("LLEG_JOINT5");
        auto p = ctl.postureTask->posture();
        ankle_reorient_target = -p[lankle_i][0]; /*XXX Hard-coded value */
        p[ankle_i][0] = ankle_reorient_target;
        p[roll_i][0] = p[lroll_i][0];
        ctl.postureTask->posture(p);
        done_setup_flatten = true;
      }
      else
      {
        double error = std::abs(ctl.robot().mbc->q[ankle_i][0] - ankle_reorient_target);
        if(error < 0.01)
        {
          ctl.postureTask->posture(ctl.robot().mbc->q);
          done_flatten = true;
          std::cout << "Finished flatten foot" << std::endl;
        }
      }
    }
    else if(!done_putdown)
    {
      if(!done_setup_putdown)
      {
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.qpsolver->robots, 0, 0.1));
        ctl.efTask->addToSolver(ctl.qpsolver->solver);
        Eigen::Vector3d lmod(0.25, -0.15, -0.1); /*XXX Hard-coded value */
        unsigned int bIdx = ctl.robot().bodyIndexByName("LLEG_LINK5");
        sva::PTransformd bpw = ctl.robot().mbc->bodyPosW[bIdx];
        sva::PTransformd targetEfPose = sva::PTransformd(Eigen::Matrix3d::Identity(), lmod)*bpw;
        ctl.efTask->set_ef_pose(targetEfPose);
        done_setup_putdown = true;
      }
      else
      {
        double error = ctl.efTask->positionTask->eval().norm();
        if(ctl.wrenches[0].first[0] > 50 || error < 0.05)
        {
          ctl.efTask->removeFromSolver(ctl.qpsolver->solver);
          ctl.postureTask->posture(ctl.robot().mbc->q);
          ctl.qpsolver->setContacts({
            mc_rbdyn::Contact(ctl.robot().surfaces.at("Butthock"), ctl.env().surfaces.at("left_seat")),
            mc_rbdyn::Contact(ctl.robot().surfaces.at("LFullSole"), ctl.env().surfaces.at("exit_platform")),
            mc_rbdyn::Contact(ctl.robot().surfaces.at("RFullSole"), ctl.env().surfaces.at("exit_platform"))
          });
          done_putdown = true;
          std::cout << "Contact found, next step" << std::endl;
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
  bool done_setup_flatten;
  bool done_flatten;
  bool done_setup_putdown;
  bool done_putdown;
  int ankle_i;
  double ankle_reorient_target;
};

}
