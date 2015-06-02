namespace mc_control
{

struct EgressMRPhaseExecution
{
public:
  /* Returns true if the phase is over */
  virtual bool run(MCEgressMRQPController & ctl) = 0;
protected:
  unsigned int timeoutIter;
};

struct EgressMRStartPhase : public EgressMRPhaseExecution
{
public:
  virtual bool run(MCEgressMRQPController & ctl) override
  {
    //return false;
    std::cout << "starting" << std::endl;
    return true;
  }
};


struct EgressRemoveRightFootPhase : public EgressMRPhaseExecution
{
  public:
    EgressRemoveRightFootPhase()
      : started(false),
        done_move_foot(false),
        done_change_knee(false),
        timeoutIter(0)
  {
  }
    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(not started)
      {
        std::cout << "Removing right foot" << std::endl;
        started = true;
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.mrqpsolver->robots, 0, 0.25));
        ctl.efTask->addToSolver(ctl.mrqpsolver->solver);
        sva::PTransformd lift(Eigen::Vector3d(-0.1, 0, 0.1)); /*XXX Hard-coded value */
        ctl.efTask->set_ef_pose(ctl.efTask->get_ef_pose()*lift);
        timeoutIter = 0;
        return false;
      }
      else if(not done_move_foot)
      {
        timeoutIter++;
        if((ctl.efTask->positionTask->eval().norm() < 1e-1 and ctl.efTask->positionTask->speed().norm() < 1e-4) or timeoutIter > 10*500)
        {
          done_move_foot = true;
          ctl.efTask->removeFromSolver(ctl.mrqpsolver->solver);
          ctl.hrp2postureTask->posture(ctl.robot().mbc->q);
          auto knee_i = ctl.robot().jointIndexByName("RLEG_JOINT3");
          auto p = ctl.hrp2postureTask->posture();
          p[knee_i][0] += 0.1;
          ctl.hrp2postureTask->posture(p);
          timeoutIter = 0;
        }
        return false;
      }
      else if(not done_change_knee)
      {
        timeoutIter++;
        if(ctl.hrp2postureTask->eval().norm() < 1e-2 or timeoutIter > 10*500)
        {
          done_change_knee = true;
          return true;
        }
        return false;
      }
      return false;
  }

  private:
    bool started;
    bool done_move_foot;
    bool done_change_knee;
    int timeoutIter;
};

struct EgressRotateLazyPhase : public EgressMRPhaseExecution
{
  public:
    EgressRotateLazyPhase()
      : started(false),
        done_rotate(false),
        done_reorient(false),
        forceIter(0)
    {
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
     if(not started)
      {
        started = true;
        ctl.hrp2postureTask->posture(ctl.robot().mbc->q);
        std::cout << "Start rotating suzan" << std::endl;
        int lazy_i = ctl.robots().robots[1].jointIndexByName("lazy_susan");
        auto p = ctl.lazyPostureTask->posture();
        p[lazy_i][0] = M_PI/2;
        return false;
      }
      else if(not done_rotate)
      {
        //Check if robot is no longer moving
        if(ctl.mrqpsolver->solver.alphaDVec(0).norm() < 5e-4)
        {
          std::cout << "Lazy susan rotation done" << std::endl;
          ctl.lazyPostureTask->posture(ctl.robots().robots[1].mbc->q);
          ctl.hrp2postureTask->posture(ctl.robot().mbc->q);
          done_rotate = true;
          ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.mrqpsolver->robots, 0, 0.25));
          ctl.efTask->addToSolver(ctl.mrqpsolver->solver);
          Eigen::Vector3d lift(0., 0, -0.05); /*XXX Hard-coded value */
          int rfindex = ctl.robot().bodyIndexByName("RLEG_LINK5");
          Eigen::Matrix3d & rrot = ctl.robot().mbc->bodyPosW[rfindex].rotation();
          Eigen::Vector3d rfrpy = rrot.eulerAngles(2,1,0);
          int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
          Eigen::Matrix3d& rot = ctl.robot().mbc->bodyPosW[lfindex].rotation();
          Eigen::Vector3d rpy = rot.eulerAngles(2, 1, 0);
          Eigen::Matrix3d target = sva::RotZ(M_PI + rfrpy(0))*sva::RotY(rpy(1))*sva::RotX(rpy(2));
          ctl.efTask->positionTask->position((ctl.efTask->get_ef_pose().translation() + lift));
          ctl.efTask->orientationTask->orientation(target);
          timeoutIter = 0;
        }
        return false;
      }
      else if(not done_reorient)
      {
        timeoutIter++;
        if((ctl.efTask->positionTask->eval().norm() < 1e-1 and ctl.efTask->positionTask->speed().norm() < 1e-4 and ctl.efTask->orientationTask->eval().norm() < 1e-2 and ctl.efTask->orientationTask->speed().norm() < 1e-4) or timeoutIter > 15*500)
        {
          ctl.hrp2postureTask->posture(ctl.robot().mbc->q);
          ctl.efTask->removeFromSolver(ctl.mrqpsolver->solver);
          ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.robots(), 0, 0.1));
          ctl.efTask->addToSolver(ctl.mrqpsolver->solver);
          sva::PTransformd down(Eigen::Vector3d(-0.0,0.0, -0.3)); /*XXX Hard-coded*/
          ctl.efTask->positionTask->position((ctl.efTask->get_ef_pose()*down).translation());
          forceIter = 0;
          forceStart = ctl.wrenches[0].first[2];
          timeoutIter = 0;
          std::cout << "Reoriented the right foot" << std::endl;
          done_reorient = true;
        }
        return false;
      }
      else
      {
        timeoutIter++;
        if(ctl.wrenches[0].first[2] > forceStart + 150)
        {
          std::cout << "Contact force triggered" << std::endl;
          forceIter++;
        }
        else
        {
          forceIter = 0;
        }
        if(forceIter > 40 or timeoutIter > 15*500)
        {
          ctl.efTask->removeFromSolver(ctl.mrqpsolver->solver);
          ctl.hrp2postureTask->posture(ctl.robot().mbc->q);
          std::cout << "Found contact on right foot" << std::endl;
          return true;
        }
        return false;
      }
    }

  private:
    bool started;
    bool done_rotate;
    bool done_reorient;
    unsigned int forceIter;
    double forceStart;
};

struct EgressRemoveLeftFootPhase : public EgressMRPhaseExecution
{
  public:
    EgressRemoveLeftFootPhase()
      : started(false),
        done_stabilizing(false),
        done_removing(false),
        lfc_index(1),
        otherContacts()
  {
  }

   virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(not started)
      {
        std::cout << "Removing left foot" << std::endl;
        auto buttContact = ctl.egressContacts.at(0);
        Eigen::Vector3d target = buttContact.X_0_r1s(ctl.robots()).translation();
        Eigen::Vector3d cur_com = rbd::computeCoM(*(ctl.robot().mb),
                                                  *(ctl.robot().mbc));
        target(2) = cur_com(2);
        ctl.comTask->set_com(target);
        ctl.comTask->addToSolver(ctl.mrqpsolver->solver);
        started = true;
        timeoutIter = 0;
        return false;
      }
      else if(not done_stabilizing)
      {
        if(ctl.comTask->comTask->speed().norm() < 1e-3
            and ctl.comTask->comTask->eval().norm() < 1e-1)
        {
          done_stabilizing = true;
          ctl.hrp2postureTask->posture(ctl.robot().mbc->q);
          ctl.comTask->removeFromSolver(ctl.mrqpsolver->solver);
          ctl.efTask->removeFromSolver(ctl.mrqpsolver->solver);
          ctl.efTask.reset(new mc_tasks::EndEffectorTask("LLEG_LINK5",
                                                         ctl.mrqpsolver->robots,
                                                         ctl.mrqpsolver->robots.robotIndex, 0.25));

          int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
          sva::PTransformd lift(Eigen::Vector3d(0.05, 0, 0.1));

          ctl.efTask->positionTask->position((lift*ctl.robot().mbc->bodyPosW[lfindex]).translation());

          //Free movement along z axis
          mc_rbdyn::MRContact& lfc = ctl.egressContacts.at(lfc_index);
          //std::copy(ctl.egressContacts.begin(), ctl.egressContacts.begin() + lfc_index, otherContacts.end());
          //std::copy(ctl.egressContacts.begin() + lfc_index + 1, ctl.egressContacts.end(), otherContacts.end());
          otherContacts.push_back(ctl.egressContacts.at(0));
          otherContacts.push_back(ctl.egressContacts.at(2));
          otherContacts.push_back(mc_rbdyn::MRContact(0, 1,
                                    ctl.robot().surfaces.at("RFullSole"),
                                    ctl.robots().robots[1].surfaces.at("left_floor")));
          ctl.mrqpsolver->setContacts(otherContacts);

          tasks::qp::ContactId cId = lfc.contactId(ctl.robots().robots);
          Eigen::MatrixXd dof(6,6);
          dof.setIdentity();
          dof(5, 5) = 0;
          auto constr = dynamic_cast<tasks::qp::ContactConstr*>(ctl.hrp2contactConstraint.contactConstr.get());
          if(constr == 0)
            std::cout << "NOPE NOPE" << std::endl;
          else
            constr->addDofContact(cId, dof);

          ctl.efTask->addToSolver(ctl.mrqpsolver->solver);
        }
        return false;
      }
      else
      {
        if(not done_removing)
        {
          timeoutIter++;
          if((ctl.efTask->positionTask->eval().norm() < 1e-2
              and ctl.efTask->positionTask->speed().norm() < 1e-4)
              or timeoutIter > 15*500)
          {
            done_removing = true;
            ctl.mrqpsolver->setContacts(otherContacts);
            ctl.hrp2postureTask->posture(ctl.robot().mbc->q);
            ctl.efTask->removeFromSolver(ctl.mrqpsolver->solver);
            return true;
          }
          return false;
        }
      return false;
      }
    }

  private:
    bool started;
    bool done_stabilizing;
    bool done_removing;
    size_t lfc_index;
    std::vector<mc_rbdyn::MRContact> otherContacts;
};

struct EgressReplaceLeftFootPhase : public EgressMRPhaseExecution
{
  public:
    EgressReplaceLeftFootPhase()
      : started(false),
        done_rotating(false),
        done_contacting(false)
    {
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(not started)
      {
        std::cout << "Replacing left foot" << std::endl;
        started = true;
        sva::PTransformd move(Eigen::Vector3d(0.1, 0., 0.));
        int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
        Eigen::Matrix3d& rot = ctl.robot().mbc->bodyPosW[lfindex].rotation();
        Eigen::Vector3d rpy = rot.eulerAngles(2, 1, 0);
        //Eigen::Vector3d rpy_body = ctl.robot().mbc->bodyPosW[0].rotation().eulerAngles(2, 1, 0);
        Eigen::Matrix3d target = sva::RotZ(-M_PI/2)*
                                 sva::RotY(rpy(1))*
                                 sva::RotX(rpy(2));
        ctl.efTask->positionTask->position((move*ctl.efTask->get_ef_pose()).translation());
        ctl.efTask->orientationTask->orientation(target);
        timeoutIter = 0;
        return false;
       }
       else if(not done_rotating)
       {
          timeoutIter++;
          if((ctl.efTask->orientationTask->eval().norm() < 1e-2
              and ctl.efTask->orientationTask->speed().norm() < 1e-4)
              or timeoutIter > 15*500)
          {
            done_rotating = true;
            ctl.efTask->removeFromSolver(ctl.mrqpsolver->solver);
            ctl.efTask.reset(new mc_tasks::EndEffectorTask("LLEG_LINK5",
                                                           ctl.mrqpsolver->robots,
                                                           ctl.mrqpsolver->robots.robotIndex, 0.1));
            int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            sva::PTransformd lower(Eigen::Vector3d(0, 0, -0.2));
            ctl.efTask->positionTask->position((lower*ctl.robot().mbc->bodyPosW[lfindex]).translation());
            timeoutIter = 0;
            forceIter = 0;
            forceStart = ctl.wrenches[1].first[2];
            std::cout << "Going to contact" << std::endl;
          }
          return false;
        }
        else if(not done_contacting)
        {
          timeoutIter++;
          if(ctl.wrenches[1].first[2] > forceStart + 150)
          {
            std::cout << "Contact force triggered" << std::endl;
            forceIter++;
          }
          else
          {
            forceIter = 0;
          }
          if(forceIter > 40 or timeoutIter > 15*500)
          {
            done_contacting = true;
            ctl.mrqpsolver->setContacts(ctl.egressContacts);
            std::cout << "Done moving left foot" << std::endl;
            return true;
          }
          return false;
        }
        return false;
      }

  private:
    bool started;
    bool done_rotating;
    bool done_contacting;
    unsigned int forceIter;
    double forceStart;
};

struct EgressPlaceRightFootPhase : public EgressMRPhaseExecution
{
  public:
    EgressPlaceRightFootPhase()
      : started(false),
        done_lifting(false),
        done_rotating(false),
        done_contacting(false)
    {
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(not started)
      {
        std::cout << "Replacing right foot" << std::endl;
        ctl.efTask->removeFromSolver(ctl.mrqpsolver->solver);

        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5",
                                                       ctl.mrqpsolver->robots,
                                                       ctl.mrqpsolver->robots.robotIndex, 0.25));

        int lfindex = ctl.robot().bodyIndexByName("RLEG_LINK5");
        sva::PTransformd lift(Eigen::Vector3d(0, 0., 0.2));

        ctl.efTask->positionTask->position((lift*ctl.robot().mbc->bodyPosW[lfindex]).translation());

        ctl.efTask->addToSolver(ctl.mrqpsolver->solver);

        started = true;
        std::cout << "Done starting rfplacement" << std::endl;
        timeoutIter = 0;
        return false;
      }
      else
      {
        timeoutIter++;
        if(not done_lifting)
        {
          if((ctl.efTask->positionTask->eval().norm() < 1e-2
              and ctl.efTask->positionTask->speed().norm() < 1e-4)
              or timeoutIter > 15*500)
          {
            done_lifting = true;
            int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            Eigen::Matrix3d& rot = ctl.robot().mbc->bodyPosW[lfindex].rotation();
            Eigen::Vector3d rpy = rot.eulerAngles(2, 1, 0);
            Eigen::Matrix3d target = sva::RotZ(-M_PI/2)*
                                     sva::RotY(rpy(1))*
                                     sva::RotX(rpy(2));
            ctl.efTask->orientationTask->orientation(target);
            timeoutIter = 0;
            std::cout << "Modified orientation" << std::endl;
          }
          return false;
        }
        else if(not done_rotating)
        {
          if((ctl.efTask->orientationTask->eval().norm() < 1e-1
              and ctl.efTask->orientationTask->speed().norm() < 1e-4)
              or timeoutIter > 15*500)
          {
            done_rotating = true;
            int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            sva::PTransformd interfeet(Eigen::Vector3d(0, -0.2, 0.));
            ctl.efTask->positionTask->position((interfeet*ctl.robot().mbc->bodyPosW[lfindex]).translation());
            timeoutIter = 0;
          }
          return false;
        }
        else if(not done_contacting)
        {
          if((ctl.efTask->positionTask->eval().norm() < 1e-2
              and ctl.efTask->positionTask->speed().norm() < 1e-4)
              or timeoutIter > 15*500)
          {
            done_contacting = true;
            mc_rbdyn::Robot& polaris = ctl.robots().robots[1];
            ctl.egressContacts.emplace_back(ctl.robots().robotIndex, 1,
                                            ctl.robot().surfaces.at("RFullSole"),
                                            polaris.surfaces.at("exit_platform"));
            ctl.mrqpsolver->setContacts(ctl.egressContacts);
            timeoutIter = 0;
            return true;
          }
          return false;
        }
        return false;
      }
    }

  private:
    bool started;
    bool done_lifting;
    bool done_rotating;
    bool done_contacting;
};

struct EgressRemoveRightGripperPhase : public EgressMRPhaseExecution
{
  public:
    EgressRemoveRightGripperPhase()
      : started(false),
        done_removing(false),
        done_posture(false),
        rgc_index(2),
        otherContacts()
    {
      std::cout << "In egress remove right gripper phase" << std::endl;
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(not started)
      {
        std::cout << "Removing right gripper" << std::endl;
        ctl.efTask->removeFromSolver(ctl.mrqpsolver->solver);

        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6",
                                                       ctl.mrqpsolver->robots,
                                                       ctl.mrqpsolver->robots.robotIndex, 0.25));

        int rgindex = ctl.robot().bodyIndexByName("RARM_LINK6");
        sva::PTransformd lift(Eigen::Vector3d(0, 0, 0.1));

        ctl.efTask->positionTask->position((lift*ctl.robot().mbc->bodyPosW[rgindex]).translation());

        ctl.efTask->addToSolver(ctl.mrqpsolver->solver);

        //Free movement along z axis
        mc_rbdyn::MRContact& rgc = ctl.egressContacts.at(rgc_index);
        otherContacts.push_back(ctl.egressContacts.at(0));
        otherContacts.push_back(ctl.egressContacts.at(2));
        otherContacts.push_back(ctl.egressContacts.at(3));

        tasks::qp::ContactId cId = rgc.contactId(ctl.robots().robots);
        Eigen::MatrixXd dof(6,6);
        dof.setIdentity();
        dof(5, 5) = 0;
        auto constr = dynamic_cast<tasks::qp::ContactConstr*>(ctl.hrp2contactConstraint.contactConstr.get());
        if(constr == 0)
          std::cout << "Fuck you" << std::endl;
        else
          constr->addDofContact(cId, dof);

        ctl.efTask->addToSolver(ctl.mrqpsolver->solver);

        started = true;
        return false;
      }
      else
      {
        if(not done_removing)
        {
          if(ctl.efTask->positionTask->eval().norm() < 1e-2
              and ctl.efTask->positionTask->speed().norm() < 1e-4)
          {
            done_removing = true;
            auto q = ctl.robot().mbc->q;
            int chest_i = ctl.robot().jointIndexByName("CHEST_JOINT0");
            q[chest_i][0] = 0;
            ctl.hrp2postureTask->posture(q);
            ctl.efTask->removeFromSolver(ctl.mrqpsolver->solver);
          }
          return false;
        }
        else if(not done_posture)
        {
          if(ctl.mrqpsolver->solver.alphaDVec(0).norm() < 5e-3)
          {
            done_posture = true;
            return true;
          }
          return false;
        }
        else
          return false;
      }
    }

  private:
    bool started;
    bool done_removing;
    bool done_posture;
    size_t rgc_index;
    std::vector<mc_rbdyn::MRContact> otherContacts;
};

struct EgressMRStandupPhase : public EgressMRPhaseExecution
{
  public:
    EgressMRStandupPhase()
      : started(false),
        done_standup(false),
        otherContacts()
    {
      std::cout << "In egress standup phase" << std::endl;
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(not started)
      {
        std::cout << "Starting standup" << std::endl;
        ctl.efTask->removeFromSolver(ctl.mrqpsolver->solver);
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("BODY", ctl.mrqpsolver->robots, 0));
        int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
        int rfindex = ctl.robot().bodyIndexByName("RLEG_LINK5");
        const auto & lfPose = ctl.robot().mbc->bodyPosW[lfindex];
        const auto & rfPose = ctl.robot().mbc->bodyPosW[rfindex];
        Eigen::Vector3d bodyTarget = (lfPose.translation() + rfPose.translation())/2;
        bodyTarget(2) = bodyTarget(2) + 0.76 - 0.15;
        ctl.efTask->positionTask->position(bodyTarget);
        ctl.efTask->orientationTask->orientation(lfPose.rotation());
        ctl.efTask->addToSolver(ctl.mrqpsolver->solver);
        otherContacts.push_back(ctl.egressContacts.at(1));
        otherContacts.push_back(ctl.egressContacts.at(2));
        otherContacts.push_back(ctl.egressContacts.at(3));
        ctl.mrqpsolver->setContacts(otherContacts);
        timeoutIter = 0;
        started = true;
      }
      else if(not done_standup)
      {
        ++timeoutIter;
        if((ctl.efTask->positionTask->eval().norm() < 1e-1 and ctl.efTask->orientationTask->speed().norm() < 1e-4 and ctl.efTask->positionTask->speed().norm() < 1e-4) or timeoutIter > 15*500)
        {
          ctl.hrp2postureTask->posture(ctl.robot().mbc->q);
          ctl.efTask->removeFromSolver(ctl.mrqpsolver->solver);
          done_standup = true;
          std::cout << "Finished standup" << std::endl;
          return true;
        }
      }
      return false;
    }

  private:
    bool started;
    bool done_standup;
    std::vector<mc_rbdyn::MRContact> otherContacts;
};

}
