namespace mc_control
{

struct EgressMRPhaseExecution
{
public:
  /* Returns true if the phase is over */
  virtual bool run(MCEgressMRQPController & ctl) = 0;
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

struct EgressRotateLazyPhase : public EgressMRPhaseExecution
{
  public:
    EgressRotateLazyPhase()
      : started(false)
    {
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(not started)
      {
        std::cout << "Start rotating suzan" << std::endl;
        int lazy_i = ctl.robots().robots[1].jointIndexByName("lazy_susan");
        auto p = ctl.lazyPostureTask->posture();
        p[lazy_i][0] = 3*M_PI/4;
        ctl.lazyPostureTask->posture(p);
        started = true;
        return false;
      }

      //Check if robot is no longer moving
      if(ctl.mrqpsolver->solver.alphaDVec(0).norm() < 5e-3)
      {
        std::cout << "Done rotating suzan" << std::endl;
        return true;
      }
      else
      {
        return false;
      }
    }

  private:
    bool started;
};

struct EgressReplaceLeftFootPhase : public EgressMRPhaseExecution
{
  public:
    EgressReplaceLeftFootPhase()
      : started(false),
        done_removing(false),
        done_rotating(false),
        lfc_index(1),
        otherContacts()
    {
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(not started)
      {
        std::cout << "Replacing left foot" << std::endl;
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("LLEG_LINK5",
                                                       ctl.mrqpsolver->robots,
                                                       ctl.mrqpsolver->robots.robotIndex));

        int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
        sva::PTransformd lift(Eigen::Vector3d(0, 0, 0.1));

        ctl.efTask->positionTask->position((lift*ctl.robot().mbc->bodyPosW[lfindex]).translation());

        //Free movement along z axis
        mc_rbdyn::MRContact& lfc = ctl.egressContacts.at(lfc_index);
        //std::copy(ctl.egressContacts.begin(), ctl.egressContacts.begin() + lfc_index, otherContacts.end());
        //std::copy(ctl.egressContacts.begin() + lfc_index + 1, ctl.egressContacts.end(), otherContacts.end());
        otherContacts.push_back(ctl.egressContacts.at(0));
        otherContacts.push_back(ctl.egressContacts.at(2));

        tasks::qp::ContactId cId = lfc.contactId(ctl.robots().robots);
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
            ctl.mrqpsolver->setContacts(otherContacts);
            sva::PTransformd move(Eigen::Vector3d(-0.1, 0., 0.));
            int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            Eigen::Matrix3d& rot = ctl.robot().mbc->bodyPosW[lfindex].rotation();
            Eigen::Vector3d rpy = rot.eulerAngles(2, 1, 0);
            //Eigen::Vector3d rpy_body = ctl.robot().mbc->bodyPosW[0].rotation().eulerAngles(2, 1, 0);
            Eigen::Matrix3d target = sva::RotZ(-M_PI/2)*
                                     sva::RotY(rpy(1))*
                                     sva::RotX(rpy(2));
            ctl.efTask->orientationTask->orientation(target);
            std::cout << "Modified orientation" << std::endl;
          }
          return false;
        }
        else if(not done_rotating)
        {
          if(ctl.efTask->orientationTask->eval().norm() < 1e-2
              and ctl.efTask->orientationTask->speed().norm() < 1e-4)
          {
            done_rotating = true;
            int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            sva::PTransformd lower(Eigen::Vector3d(0, 0, -0.1));
            ctl.efTask->positionTask->position((lower*ctl.robot().mbc->bodyPosW[lfindex]).translation());
          }
          return false;
        }
        else if(not done_contacting)
        {
          if(ctl.efTask->positionTask->eval().norm() < 1e-2
              and ctl.efTask->positionTask->speed().norm() < 1e-4)
          {
            done_contacting = true;
            ctl.mrqpsolver->setContacts(ctl.egressContacts);
            return true;
          }
          return false;
        }
        return false;
      }
    }

  private:
    bool started;
    bool done_removing;
    bool done_rotating;
    bool done_contacting;
    size_t lfc_index;
    std::vector<mc_rbdyn::MRContact> otherContacts;
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
                                                       ctl.mrqpsolver->robots.robotIndex));

        int lfindex = ctl.robot().bodyIndexByName("RLEG_LINK5");
        sva::PTransformd lift(Eigen::Vector3d(0, 0, 0.1));

        ctl.efTask->positionTask->position((lift*ctl.robot().mbc->bodyPosW[lfindex]).translation());

        ctl.efTask->addToSolver(ctl.mrqpsolver->solver);

        started = true;
        std::cout << "Done starting rfplacement" << std::endl;
        return false;
      }
      else
      {
        if(not done_lifting)
        {
          if(ctl.efTask->positionTask->eval().norm() < 1e-2
              and ctl.efTask->positionTask->speed().norm() < 1e-4)
          {
            done_lifting = true;
            int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            Eigen::Matrix3d& rot = ctl.robot().mbc->bodyPosW[lfindex].rotation();
            Eigen::Vector3d rpy = rot.eulerAngles(2, 1, 0);
            Eigen::Matrix3d target = sva::RotZ(-M_PI/2)*
                                     sva::RotY(rpy(1))*
                                     sva::RotX(rpy(2));
            ctl.efTask->orientationTask->orientation(target);
            std::cout << "Modified orientation" << std::endl;
          }
          return false;
        }
        else if(not done_rotating)
        {
          if(ctl.efTask->orientationTask->eval().norm() < 1e-1
              and ctl.efTask->orientationTask->speed().norm() < 1e-4)
          {
            done_rotating = true;
            int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            sva::PTransformd interfeet(Eigen::Vector3d(0, -0.2, 0.));
            ctl.efTask->positionTask->position((interfeet*ctl.robot().mbc->bodyPosW[lfindex]).translation());
          }
          return false;
        }
        else if(not done_contacting)
        {
          if(ctl.efTask->positionTask->eval().norm() < 1e-2
              and ctl.efTask->positionTask->speed().norm() < 1e-4)
          {
            done_contacting = true;
            mc_rbdyn::Robot& polaris = ctl.robots().robots[1];
            ctl.egressContacts.emplace_back(ctl.robots().robotIndex, 1,
                                            ctl.robot().surfaces.at("RFullSole"),
                                            polaris.surfaces.at("exit_platform"));
            ctl.mrqpsolver->setContacts(ctl.egressContacts);
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
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(not started)
      {
        std::cout << "Removing right gripper" << std::endl;
        ctl.efTask->removeFromSolver(ctl.mrqpsolver->solver);

        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6",
                                                       ctl.mrqpsolver->robots,
                                                       ctl.mrqpsolver->robots.robotIndex));

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

}
