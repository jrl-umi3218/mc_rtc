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
  virtual bool run(MCEgressMRQPController &) override
  {
    LOG_INFO("starting")
    return false;
    //return true;
  }
};

struct EgressRotateLazyPhase : public EgressMRPhaseExecution
{
  public:
    EgressRotateLazyPhase()
      : started(false),
        done_move_foot(false),
        done_change_knee(false),
        done_rotate(false),
        done_reorient(false),
        done_putdown(false),
        forceIter(0), forceStart(0)
    {
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(!started)
      {
        started = true;
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.robots(), 0, 0.25));
        ctl.solver().addTask(ctl.efTask);
        Eigen::Vector3d lift(-0.05, 0, 0.05); /*XXX Hard-coded value */
        ctl.efTask->positionTask->position((ctl.efTask->get_ef_pose().translation() + lift));
        timeoutIter = 0;
        return false;
      }
      else if(!done_move_foot)
      {
        timeoutIter++;
        if((ctl.efTask->positionTask->eval().norm() < 1e-1 && ctl.efTask->positionTask->speed().norm() < 1e-4) || timeoutIter > 10*500)
        {
          done_move_foot = true;
          ctl.solver().removeTask(ctl.efTask);
          ctl.postureTask->posture(ctl.robot().mbc().q);
          auto knee_i = ctl.robot().jointIndexByName("RLEG_JOINT3");
          auto p = ctl.postureTask->posture();
          p[knee_i][0] += 0.1;
          ctl.postureTask->posture(p);
          timeoutIter = 0;
        }
        return false;
      }
      else if(!done_change_knee)
      {
        timeoutIter++;
        if(ctl.postureTask->eval().norm() < 1e-2 || timeoutIter > 10*500)
        {
          done_change_knee = true;
          ctl.postureTask->posture(ctl.robot().mbc().q);
          LOG_INFO("Start rotating the leg")
          //int lazy_i = ctl.robots().robots[1].jointIndexByName("lazy_susan");
          //auto p = ctl.lazyPostureTask->posture();
          //p[lazy_i][0] = M_PI/6;
          //ctl.lazyPostureTask->posture(p);
          ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.robots(), 0, 0.25));
          ctl.solver().addTask(ctl.efTask);
          Eigen::Vector3d move(-0.1, 0.2, 0.0); /*XXX Hard-coded value */
          ctl.efTask->positionTask->position((ctl.efTask->get_ef_pose().translation() + move));
          Eigen::Matrix3d change = sva::RotZ(20*M_PI/180);
          ctl.efTask->orientationTask->orientation(ctl.efTask->get_ef_pose().rotation()*change);
          timeoutIter = 0;
        }
        return false;
      }
      else if(!done_rotate)
      {
        //Check if robot is no longer moving
        timeoutIter++;
        if((ctl.efTask->positionTask->eval().norm() < 1e-1 && ctl.efTask->positionTask->speed().norm() < 1e-4 && ctl.efTask->orientationTask->eval().norm() < 1e-2 && ctl.efTask->orientationTask->speed().norm() < 1e-4) || timeoutIter > 15*500)
        {
          LOG_INFO("Lazy susan rotation done")
          ctl.lazyPostureTask->posture(ctl.robots().robot(1).mbc().q);
          ctl.postureTask->posture(ctl.robot().mbc().q);
          done_rotate = true;
          ctl.solver().removeTask(ctl.efTask);
          ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.robots(), 0, 0.25));
          ctl.solver().addTask(ctl.efTask);
          Eigen::Vector3d lift(0., 0, 0.0); /*XXX Hard-coded value */
          //int rfindex = ctl.robot().bodyIndexByName("RLEG_LINK5");
          //Eigen::Matrix3d & rrot = ctl.robot().mbc().bodyPosW[rfindex].rotation();
          //Eigen::Vector3d rfrpy = rrot.eulerAngles(2,1,0);
          unsigned int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
          Eigen::Matrix3d& rot = ctl.robot().mbc().bodyPosW[lfindex].rotation();
          Eigen::Vector3d rpy = rot.eulerAngles(2, 1, 0);
          auto target = sva::RotZ(M_PI)*sva::RotY(rpy(1))*sva::RotX(rpy(2));
          ctl.efTask->positionTask->position((ctl.efTask->get_ef_pose().translation() + lift));
          ctl.efTask->orientationTask->orientation(target);
          timeoutIter = 0;
        }
        return false;
      }
      else if(!done_reorient)
      {
        timeoutIter++;
        if((ctl.efTask->positionTask->eval().norm() < 1e-1 && ctl.efTask->positionTask->speed().norm() < 1e-4 && ctl.efTask->orientationTask->eval().norm() < 1e-2 && ctl.efTask->orientationTask->speed().norm() < 1e-4) || timeoutIter > 15*500)
        {
          ctl.postureTask->posture(ctl.robot().mbc().q);
          ctl.solver().removeTask(ctl.efTask);
          ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.robots(), 0, 0.1));
          ctl.solver().addTask(ctl.efTask);
          sva::PTransformd down(Eigen::Vector3d(-0.0,0.0, -0.3)); /*XXX Hard-coded*/
          ctl.efTask->positionTask->position((ctl.efTask->get_ef_pose()*down).translation());
          forceIter = 0;
          forceStart = ctl.robot().forceSensor("RightFootForceSensor").force()[2];
          timeoutIter = 0;
          LOG_INFO("Reoriented the right foot")
          done_reorient = true;
        }
        return false;
      }
      else if(!done_putdown)
      {
        timeoutIter++;
        if(ctl.robot().forceSensor("RightFootForceSensor").force()[2] > forceStart + 50)
        {
          LOG_INFO("Contact force triggered")
          forceIter++;
        }
        else
        {
          forceIter = 0;
        }
        if(forceIter > 40 || timeoutIter > 15*500)
        {
          done_putdown = true;
          ctl.solver().removeTask(ctl.efTask);
          ctl.postureTask->posture(ctl.robot().mbc().q);
          LOG_INFO("Found contact on right foot")
          ctl.egressContacts.push_back(mc_rbdyn::Contact(ctl.robots(), 0, 1,
                                       "RFullSole", "left_floor"));
          ctl.solver().setContacts(ctl.egressContacts);
          LOG_INFO("Phase over, ready for next")
          //return true;
        }
        return false;
      }
      else
      {
        return false;
      }
    }

  private:
    bool started;
    bool done_move_foot;
    bool done_change_knee;
    bool done_rotate;
    bool done_reorient;
    bool done_putdown;
    unsigned int forceIter;
    double forceStart;
};

struct EgressReplaceLeftFootPhase : public EgressMRPhaseExecution
{
  public:
    EgressReplaceLeftFootPhase()
      : started(false),
        done_removing(false),
        done_rotating(false),
        done_contacting(false),
        lfc_index(0),
        forceIter(0), forceStart(0),
        com_multiplier(0.1),
        otherContacts()
    {
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(!started)
      {
        LOG_INFO("Replacing left foot")
        ctl.solver().removeTask(ctl.efTask);
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("LLEG_LINK5",
                                                       ctl.robots(),
                                                       ctl.robots().robotIndex(), 0.25));

        unsigned int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
        sva::PTransformd lift(Eigen::Vector3d(0.0, 0, 0.1));

        ctl.efTask->positionTask->position((lift*ctl.robot().mbc().bodyPosW[lfindex]).translation());

        //Free movement along z axis
        mc_rbdyn::Contact& lfc = ctl.egressContacts.at(lfc_index);
        //std::copy(ctl.egressContacts.begin(), ctl.egressContacts.begin() + lfc_index, otherContacts.end());
        //std::copy(ctl.egressContacts.begin() + lfc_index + 1, ctl.egressContacts.end(), otherContacts.end());
        otherContacts.push_back(ctl.egressContacts.at(1));
        otherContacts.push_back(ctl.egressContacts.at(2));

        for(auto c: otherContacts)
        {
          LOG_INFO(c.r1Surface()->name() << " / " << c.r2Surface()->name())
        }

        ctl.solver().setContacts(ctl.egressContacts);

        tasks::qp::ContactId cId = lfc.contactId(ctl.robots());
        Eigen::MatrixXd dof(6,6);
        dof.setIdentity();
        dof(5, 5) = 0;
        auto constr = dynamic_cast<tasks::qp::ContactConstr*>(ctl.contactConstraint.contactConstr.get());
        if (constr == 0)
        {
          LOG_INFO("NOPE NOPE")
        }
        else
        {
          constr->addDofContact(cId, dof);
        }

        ctl.solver().addTask(ctl.efTask);

        started = true;
        timeoutIter = 0;
        return false;
      }
      else
      {
        if(!done_removing)
        {
          timeoutIter++;
          if((ctl.efTask->positionTask->eval().norm() < 1e-2
              && ctl.efTask->positionTask->speed().norm() < 1e-4)
              || timeoutIter > 15*500)
          {
            done_removing = true;
            ctl.addCollision(mc_rbdyn::Collision("RLEG_LINK4", "exit_platform", 0.05, 0.01, 0.));
            //ctl.addCollision(mc_rbdyn::Collision("RLEG_LINK3", "exit_platform", 0.05, 0.01, 0.));
            ctl.solver().setContacts(otherContacts);
            Eigen::Vector3d move(-0.1, 0.35, 0.);
            //int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            unsigned int rfindex = ctl.robot().bodyIndexByName("RLEG_LINK5");
            const sva::PTransformd& rfpos = ctl.robot().mbc().bodyPosW[rfindex];
            //Eigen::Matrix3d& rot = ctl.robot().mbc().bodyPosW[lfindex].rotation();
            //Eigen::Vector3d rpy = rot.eulerAngles(2, 1, 0);
            //Eigen::Vector3d rpy_body = ctl.robot().mbc().bodyPosW[0].rotation().eulerAngles(2, 1, 0);
            //Eigen::Matrix3d target = sva::RotZ(M_PI/2);
                                     //*sva::RotY(rpy(1))
                                     //*sva::RotX(rpy(2));
            ctl.efTask->positionTask->position(rfpos.translation()+move);
            ctl.efTask->orientationTask->orientation(rfpos.rotation());
            timeoutIter = 0;
            LOG_INFO("Modified orientation")
          }
          return false;
        }
        else if(!done_rotating)
        {
          timeoutIter++;
          if((ctl.efTask->positionTask->eval().norm() < 1e-2
              && ctl.efTask->positionTask->speed().norm() < 1e-4)
              || timeoutIter > 15*500)
          {
            done_rotating = true;
            //ctl.solver().removeTask(ctl.efTask);
            //ctl.efTask.reset(new mc_tasks::EndEffectorTask("LLEG_LINK5",
            //                                               ctl.robots(),
            //                                               ctl.robots().robotIndex(), 10.1));
            unsigned int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            Eigen::Vector3d lower(0, -0.1, -0.5);
            ctl.efTask->positionTask->position(lower+ctl.robot().mbc().bodyPosW[lfindex].translation());
            double w = ctl.efTask->orientationTask->weight();
            ctl.efTask->orientationTask->weight(w*100);
            //ctl.solver().addTask(ctl.efTask);

            timeoutIter = 0;
            forceIter = 0;
            forceStart = ctl.robot().forceSensor("LeftFootForceSensor").force()[2];

            w = ctl.comTask->weight();
            ctl.comTask->weight(w*com_multiplier);

            LOG_INFO("Reached contacts phase")
            ctl.egressContacts.erase(ctl.egressContacts.begin()+static_cast<int>(lfc_index));
            ctl.solver().setContacts(ctl.egressContacts);
            //ctl.egressContacts.emplace_back(ctl.robots().robotIndex, 2,
            //                                ctl.robot().surfaces.at("LFullSole"),
            //                                ctl.robots().robots[2].surfaces.at("AllGround"));
            //ctl.solver().setContacts(ctl.egressContacts);
            LOG_INFO("Set contacts")
            for(auto c : ctl.egressContacts)
            {
              LOG_INFO(c.r1Surface()->name() << " / " << c.r2Surface()->name())
            }

            //mc_rbdyn::Contact& lfc = ctl.egressContacts.back();
            //LOG_INFO("Found lfc")
            //tasks::qp::ContactId cId = lfc.contactId(ctl.robots().robots);
            //Eigen::MatrixXd dof(6,6);
            //dof.setZero();
            //dof(5, 5) = 0;
            //auto constr = dynamic_cast<tasks::qp::ContactConstr*>(ctl.contactConstraint.contactConstr.get());
            //if(constr == 0)
            //  LOG_INFO("NOPE NOPE")
            //else
            //  std::cout << "Added dof on " << lfc.r1Surface()->name << " / "
            //                               << lfc.r2Surface()->name << std::endl;
            //  constr->addDofContact(cId, dof);

            LOG_INFO("Going to contact")
          }
          return false;
        }
        else if(!done_contacting)
        {
          timeoutIter++;
          if(ctl.robot().forceSensor("LeftFootForceSensor").force()[2] > forceStart + 150)
          {
            LOG_INFO("Contact force triggered")
            forceIter++;
          }
          else
          {
            forceIter = 0;
          }
          if(forceIter > 40 || timeoutIter > 15*500)
          {
            done_contacting = true;
            LOG_INFO(ctl.robots().robot(2).surfaces().size())
            for(auto s : ctl.robots().robot(2).surfaces())
            {
              LOG_INFO(s.first)
            }
            auto constr = dynamic_cast<tasks::qp::ContactConstr*>(ctl.contactConstraint.contactConstr.get());
            if (constr == 0)
            {
              LOG_INFO("NOPE NOPE")
            }
            else
            {
              constr->resetDofContacts();
            }
            //NB : When using dof contacts, do !add twice !
            ctl.egressContacts.emplace_back(ctl.robots(), ctl.robots().robotIndex(), 2,
                                            "LFullSole", "AllGround");
            ctl.solver().setContacts(ctl.egressContacts);
            double w = ctl.comTask->weight();
            ctl.comTask->weight(w/com_multiplier);
            LOG_INFO("Done moving left foot")
            LOG_INFO("Phase finished, can transit")
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
    unsigned int forceIter;
    double forceStart;
    double com_multiplier;
    std::vector<mc_rbdyn::Contact> otherContacts;
};

struct EgressPutDownRightFootPhase : public EgressMRPhaseExecution
{
  public:
    EgressPutDownRightFootPhase()
      : started(false),
        done_removing(false),
        done_moving(false),
        done_rotating(false),
        done_contacting(false),
        forceIter(0), forceStart(0),
        otherContacts()
    {
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(!started)
      {
        LOG_INFO("Moving right foot to the ground")

        ctl.comTask->stiffness(5.);
        ctl.comTask->weight(100.);

        ctl.solver().removeTask(ctl.efTask);
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5",
                                                       ctl.robots(),
                                                       ctl.robots().robotIndex(), 0.25));

        ctl.torsoOriTask->reset();
        ctl.torsoOriTask->weight(10.);
        ctl.solver().addTask(ctl.torsoOriTask);

        unsigned int lfindex = ctl.robot().bodyIndexByName("RLEG_LINK5");
        sva::PTransformd lift(Eigen::Vector3d(0.0, 0, 0.1));

        ctl.efTask->positionTask->position((lift*ctl.robot().mbc().bodyPosW[lfindex]).translation());

        //Free movement along z axis
        auto rfc = std::find_if(ctl.egressContacts.begin(),
                                ctl.egressContacts.end(),
                                [](const mc_rbdyn::Contact & c) -> bool { return c.r1Surface()->name().compare("RFullSole") == 0; });
        //mc_rbdyn::Contact& rfc = ctl.egressContacts.at(rfc_index);

        ctl.solver().setContacts(ctl.egressContacts);

        tasks::qp::ContactId cId = (*rfc).contactId(ctl.robots());
        Eigen::MatrixXd dof(6,6);
        dof.setIdentity();
        dof(2, 2) = 0;
        dof(5, 5) = 0;
        auto constr = dynamic_cast<tasks::qp::ContactConstr*>(ctl.contactConstraint.contactConstr.get());
        if (constr == 0)
        {
          LOG_INFO("NOPE NOPE")
        }
        else
        {
          constr->addDofContact(cId, dof);
        }

        ctl.solver().addTask(ctl.efTask);

        ctl.egressContacts.erase(rfc);

        started = true;
        timeoutIter = 0;
        return false;
      }
      else
      {
        if(!done_removing)
        {
          timeoutIter++;
          if((ctl.efTask->positionTask->eval().norm() < 1e-2
              && ctl.efTask->positionTask->speed().norm() < 1e-4)
              || timeoutIter > 15*500)
          {
            done_removing = true;
            ctl.solver().setContacts(ctl.egressContacts);
            Eigen::Vector3d move(0.3, 0.2, 0);
            unsigned int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            const sva::PTransformd& lfpos = ctl.robot().mbc().bodyPosW[lfindex];
            Eigen::Vector3d target = move + lfpos.translation();
            target(2) = ctl.efTask->positionTask->position()(2);
            ctl.efTask->positionTask->position(target);
            ctl.efTask->orientationTask->orientation(lfpos.rotation());
            timeoutIter = 0;
            LOG_INFO("Modified position")
          }
          return false;
        }
        else if(!done_moving)
        {
          timeoutIter++;
          if((ctl.efTask->positionTask->eval().norm() < 1e-2
              && ctl.efTask->positionTask->speed().norm() < 1e-4)
              || timeoutIter > 15*500)
          {
            done_moving = true;
            timeoutIter = 0;
            LOG_INFO("Done moving")
          }
          return false;
        }
        else if(!done_rotating)
        {
          timeoutIter++;
          if((ctl.efTask->orientationTask->eval().norm() < 1e-2
              && ctl.efTask->orientationTask->speed().norm() < 1e-4)
              || timeoutIter > 15*500)
          {
            done_rotating = true;
            unsigned int rfindex = ctl.robot().bodyIndexByName("RLEG_LINK5");
            Eigen::Vector3d lower(0, -0.2, -0.5);

            double w = ctl.efTask->orientationTask->weight();
            ctl.efTask->orientationTask->weight(w*100);

            ctl.efTask->positionTask->position(lower+ctl.robot().mbc().bodyPosW[rfindex].translation());

            ctl.solver().setContacts(ctl.egressContacts);

            timeoutIter = 0;
            forceIter = 0;
            forceStart = ctl.robot().forceSensor("RightFootForceSensor").force()[2];
            LOG_INFO("Going to contact")
          }
          return false;
        }
        else if(!done_contacting)
        {
          timeoutIter++;
          if(ctl.robot().forceSensor("RightFootForceSensor").force()[2] > forceStart + 150)
          {
            LOG_INFO("Contact force triggered")
            forceIter++;
          }
          else
          {
            forceIter = 0;
          }
          if(forceIter > 40 || timeoutIter > 15*500)
          {
            done_contacting = true;
            auto constr = dynamic_cast<tasks::qp::ContactConstr*>(ctl.contactConstraint.contactConstr.get());
            constr->resetDofContacts();
            //NB : When using dof contacts, do !add twice !
            ctl.egressContacts.emplace_back(ctl.robots(), ctl.robots().robotIndex(), 2,
                                            "RFullSole", "AllGround");
            ctl.solver().setContacts(ctl.egressContacts);
            ctl.comTask->stiffness(1.);
            //Do !remove orientation here if we are !in skip mode
            //ctl.solver().removeTask(ctl.torsoOriTask);
            LOG_INFO("Done putting down right foot")
            //return true;
          }
          return false;
        }
        return false;
      }
    }

  private:
    bool started;
    bool done_removing;
    bool done_moving;
    bool done_rotating;
    bool done_contacting;
    unsigned int forceIter;
    double forceStart;
    std::vector<mc_rbdyn::Contact> otherContacts;
};


struct EgressReplaceRightFootPhase : public EgressMRPhaseExecution
{
  public:
    EgressReplaceRightFootPhase()
      : started(false),
        done_removing(false),
        done_premoving(false),
        done_moving(false),
        done_rotating(false),
        done_contacting(false),
        rfc_index(2),
        forceIter(0), forceStart(0),
        otherContacts()
    {
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(!started)
      {
        LOG_INFO("Replacing right foot")
        ctl.solver().removeTask(ctl.efTask);
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5",
                                                       ctl.robots(),
                                                       ctl.robots().robotIndex(), 0.25));

        unsigned int lfindex = ctl.robot().bodyIndexByName("RLEG_LINK5");
        sva::PTransformd lift(Eigen::Vector3d(0.05, 0, 0.1));

        ctl.efTask->positionTask->position((lift*ctl.robot().mbc().bodyPosW[lfindex]).translation());
        //ctl.efTask->orientationTask->orientation(sva::RotZ(-M_PI/3));

        //Free movement along z axis
        mc_rbdyn::Contact& rfc = ctl.egressContacts.at(rfc_index);
        //std::copy(ctl.egressContacts.begin(), ctl.egressContacts.begin() + lfc_index, otherContacts.end());
        //std::copy(ctl.egressContacts.begin() + lfc_index + 1, ctl.egressContacts.end(), otherContacts.end());
        otherContacts.push_back(ctl.egressContacts.at(0));
        otherContacts.push_back(ctl.egressContacts.at(1));
        ctl.solver().setContacts(ctl.egressContacts);

        tasks::qp::ContactId cId = rfc.contactId(ctl.robots());
        Eigen::MatrixXd dof(6,6);
        dof.setIdentity();
        dof(2, 2) = 0;
        dof(5, 5) = 0;
        auto constr = dynamic_cast<tasks::qp::ContactConstr*>(ctl.contactConstraint.contactConstr.get());
        if (constr == 0)
        {
          LOG_INFO("NOPE NOPE")
        }
        else
        {
          constr->addDofContact(cId, dof);
        }

        ctl.solver().addTask(ctl.efTask);

        started = true;
        timeoutIter = 0;
        return false;
      }
      else
      {
        if(!done_premoving)
        {
          timeoutIter++;
          if((ctl.efTask->positionTask->eval().norm() < 1e-2
              && ctl.efTask->positionTask->speed().norm() < 1e-4)
              || timeoutIter > 15*500)
          {
            done_premoving = true;
            ctl.solver().setContacts(otherContacts);
            Eigen::Vector3d move(-0.14, 0.0, 0);/*FIXME For safer egress, this should be based on the relative position between the right && the left foot */
            ctl.efTask->positionTask->position(ctl.efTask->positionTask->position() + move);
            timeoutIter = 0;
          }
          return false;
        }
        if(!done_removing)
        {
          timeoutIter++;
          if((ctl.efTask->positionTask->eval().norm() < 1e-2
              && ctl.efTask->positionTask->speed().norm() < 1e-4)
              || timeoutIter > 15*500)
          {
            done_removing = true;
            //ctl.solver().setContacts(otherContacts);
            Eigen::Vector3d move(0., 0.3, 0);/*FIXME For safer egress, this should be based on the relative position between the right && the left foot */
            ctl.efTask->positionTask->position(ctl.efTask->positionTask->position() + move);
            timeoutIter = 0;
            //ctl.solver().removeTask(ctl.efTask);

            //Eigen::Vector3d move(0.35, -0.15, 0.20);
            //unsigned int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            //const sva::PTransformd& lfpos = ctl.robot().mbc().bodyPosW[lfindex];
            //sva::PTransformd target(lfpos.rotation(), lfpos.translation()+move);
            //Eigen::MatrixXd wps(3,1);
            //unsigned int bindex = ctl.robot().bodyIndexByName("BODY");
            //const sva::PTransformd & bpos = ctl.robot().mbc().bodyPosW[bindex];
            //unsigned int rfindex = ctl.robot().bodyIndexByName("RLEG_LINK5");
            //const sva::PTransformd& rfpos = ctl.robot().mbc().bodyPosW[rfindex];
            //wps.col(0) = bpos.translation();
            //wps.col(0)(0) += 0.2;
            //wps.col(0)(2) = rfpos.translation()(2);
            //ctl.trajTask.reset(new mc_tasks::TrajectoryTask(ctl.robots(), 0, *(ctl.robot().surfaces.at("RFullSole").get()), target, 30.0, ctl.timeStep, 0.1, 1e2, 1));//, "TrajectoryTask", wps));
            //ctl.solver().addTask(ctl.trajTask);
            //prev_weight = ctl.efTask->orientationTask->weight();
            //ctl.efTask->orientationTask->weight(0.1);
            //auto p = ctl.postureTask->posture();
            //int rankle_i = ctl.robot().jointIndexByName("RLEG_JOINT4");
            //p[rankle_i][0] = -90*M_PI/180;
            //ctl.postureTask->posture(p);
            LOG_INFO("Modified position")
          }
          return false;
        }
        else if(!done_moving)
        {
          //ctl.trajTask->update();
          //if(ctl.trajTask->timeElapsed())
          timeoutIter++;
          if((ctl.efTask->positionTask->eval().norm() < 1e-2
              && ctl.efTask->positionTask->speed().norm() < 1e-4)
              || timeoutIter > 15*500)
          {
            done_moving = true;
            //ctl.solver().removeTask(ctl.trajTask);
            //ctl.postureTask->posture(ctl.robot().mbc().q);
            //ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", ctl.robots(), 0, 0.25));
            unsigned int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            const sva::PTransformd& lfpos = ctl.robot().mbc().bodyPosW[lfindex];
            ctl.efTask->orientationTask->orientation(lfpos.rotation());
            Eigen::Vector3d move(0.25, 0.05, 0.20);
            //unsigned int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            //const sva::PTransformd& lfpos = ctl.robot().mbc().bodyPosW[lfindex];
            ctl.efTask->positionTask->position(lfpos.translation() + move);
            timeoutIter = 0;
            LOG_INFO("Going above exit contact")
          }
          return false;
        }
        else if(!done_rotating)
        {
          timeoutIter++;
          if((ctl.efTask->orientationTask->eval().norm() < 1e-2
              && ctl.efTask->orientationTask->speed().norm() < 1e-4)
              || timeoutIter > 15*500)
          {
            done_rotating = true;
            ctl.solver().removeTask(ctl.efTask);
            ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5",
                                                           ctl.robots(),
                                                           ctl.robots().robotIndex(), 0.1));
            unsigned int lfindex = ctl.robot().bodyIndexByName("RLEG_LINK5");
            Eigen::Vector3d lower(0, 0, -0.4);
            ctl.postureTask->posture(ctl.robot().mbc().q);
            ctl.solver().addTask(ctl.efTask);
            ctl.efTask->positionTask->position(lower + ctl.robot().mbc().bodyPosW[lfindex].translation());
            ctl.solver().setContacts(ctl.egressContacts);

            //Free movement along z axis
            mc_rbdyn::Contact& rfc = ctl.egressContacts.at(rfc_index);
            otherContacts.push_back(ctl.egressContacts.at(0));
            otherContacts.push_back(ctl.egressContacts.at(1));
            ctl.solver().setContacts(ctl.egressContacts);

            tasks::qp::ContactId cId = rfc.contactId(ctl.robots());
            Eigen::MatrixXd dof(6,6);
            dof.setIdentity();
            dof(5, 5) = 0;
            auto constr = dynamic_cast<tasks::qp::ContactConstr*>(ctl.contactConstraint.contactConstr.get());
            if (constr == 0)
            {
              LOG_INFO("NOPE NOPE")
            }
            else
            {
              constr->addDofContact(cId, dof);
            }
            timeoutIter = 0;
            forceIter = 0;
            forceStart = ctl.robot().forceSensor("RightFootForceSensor").force()[2];
            LOG_INFO("Going to contact")
          }
          return false;
        }
        else if(!done_contacting)
        {
          timeoutIter++;
          if(ctl.robot().forceSensor("RightFootForceSensor").force()[2] > forceStart + 150)
          {
            LOG_INFO("Contact force triggered")
            forceIter++;
          }
          else
          {
            forceIter = 0;
          }
          if(forceIter > 40 || timeoutIter > 30*500)
          {
            done_contacting = true;
            auto constr = dynamic_cast<tasks::qp::ContactConstr*>(ctl.contactConstraint.contactConstr.get());
            constr->resetDofContacts();
            ctl.solver().setContacts(ctl.egressContacts);
            //auto p = ctl.postureTask->posture();
            //int lelbow_i = ctl.robot().jointIndexByName("LARM_JOINT3");
            //int lwrist_i = ctl.robot().jointIndexByName("LARM_JOINT5");
            //p[lelbow_i][0] = 0.0;
            //p[lwrist_i][0] = 0.0;
            //ctl.postureTask->posture(p);
            LOG_INFO("Done moving right foot")
            LOG_INFO("Phase finished, can transit")
            //return true;
          }
          return false;
        }
        return false;
      }
    }

  private:
    bool started;
    bool done_removing;
    bool done_premoving;
    bool done_moving;
    bool done_rotating;
    bool done_contacting;
    size_t rfc_index;
    unsigned int forceIter;
    double forceStart;
    std::vector<mc_rbdyn::Contact> otherContacts;
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
      if(!started)
      {
        LOG_INFO("Replacing right foot")
        ctl.solver().removeTask(ctl.efTask);

        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5",
                                                       ctl.robots(),
                                                       ctl.robots().robotIndex(), 0.25));

        unsigned int lfindex = ctl.robot().bodyIndexByName("RLEG_LINK5");
        sva::PTransformd lift(Eigen::Vector3d(0, 0., 0.1));

        ctl.efTask->positionTask->position((lift*ctl.robot().mbc().bodyPosW[lfindex]).translation());

        ctl.solver().addTask(ctl.efTask);

        started = true;
        LOG_INFO("Done starting rfplacement")
        timeoutIter = 0;
        return false;
      }
      else
      {
        timeoutIter++;
        if(!done_lifting)
        {
          if((ctl.efTask->positionTask->eval().norm() < 1e-2
              && ctl.efTask->positionTask->speed().norm() < 1e-4)
              || timeoutIter > 15*500)
          {
            done_lifting = true;
            unsigned int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            Eigen::Matrix3d& rot = ctl.robot().mbc().bodyPosW[lfindex].rotation();
            Eigen::Vector3d rpy = rot.eulerAngles(2, 1, 0);
            auto target = sva::RotZ(-M_PI/2)*
                            sva::RotY(rpy(1))*
                            sva::RotX(rpy(2));
            ctl.efTask->orientationTask->orientation(target);
            timeoutIter = 0;
            LOG_INFO("Modified orientation")
          }
          return false;
        }
        else if(!done_rotating)
        {
          if((ctl.efTask->orientationTask->eval().norm() < 1e-1
              && ctl.efTask->orientationTask->speed().norm() < 1e-4)
              || timeoutIter > 15*500)
          {
            done_rotating = true;
            unsigned int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
            sva::PTransformd interfeet(Eigen::Vector3d(0, -0.2, 0.));
            ctl.efTask->positionTask->position((interfeet*ctl.robot().mbc().bodyPosW[lfindex]).translation());
            timeoutIter = 0;
          }
          return false;
        }
        else if(!done_contacting)
        {
          if((ctl.efTask->positionTask->eval().norm() < 1e-2
              && ctl.efTask->positionTask->speed().norm() < 1e-4)
              || timeoutIter > 15*500)
          {
            done_contacting = true;
            ctl.egressContacts.emplace_back(ctl.robots(), ctl.robots().robotIndex(), 1,
                                            "RFullSole", "exit_platform");
            ctl.solver().setContacts(ctl.egressContacts);
            timeoutIter = 0;
            LOG_INFO("Phase finished, can transit")
            //return true;
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

struct EgressOpenRightGripperPhase : public EgressMRPhaseExecution
{
  public:
    EgressOpenRightGripperPhase()
      : started(false),
        done_opening(false)
  {
  }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(!started)
      {
        started = true;
        LOG_INFO("Opening gripper")
        return false;
      }
      else if(!done_opening)
      {
        if(ctl.grippers["r_gripper"]->percentOpen[0] >= 1.)
        {
          ctl.grippers["r_gripper"]->percentOpen[0] = 1.;
          done_opening = true;
          return true;
        }
        else
        {
          ctl.grippers["r_gripper"]->percentOpen[0] += 0.005;
          return false;
        }
      }
     else
     {
       LOG_INFO("We should never be here")
       return false;
     }
  }



  private:
    bool started;
    bool done_opening;
};

struct EgressRemoveRightGripperPhase : public EgressMRPhaseExecution
{
  public:
    EgressRemoveRightGripperPhase(int max_wiggles, double dist, double deg)
      : started(false),
        done_rotating(false),
        done_removing(false),
        wiggles_(0),
        maxWiggles_(max_wiggles),
        dist_(dist),
        rot(sva::RotX(deg*M_PI/180)),
        birot(sva::RotX(2*deg*M_PI/180))
    {
      LOG_INFO("In egress remove right gripper phase")
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(!started)
      {
        LOG_INFO("Removing right gripper")
        ctl.solver().removeTask(ctl.efTask);

        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6",
                                                       ctl.robots(),
                                                       ctl.robots().robotIndex(),
                                                       12., 1000));

        unsigned int rgindex = ctl.robot().bodyIndexByName("RARM_LINK6");

        ctl.efTask->orientationTask->orientation((sva::PTransformd(rot)*ctl.robot().mbc().bodyPosW[rgindex]).rotation());

        //Free movement along z axis
        auto rgc = std::find_if(ctl.egressContacts.begin(),
                                ctl.egressContacts.end(),
                                [](const mc_rbdyn::Contact & c) -> bool { return c.r1Surface()->name().compare("RightGripper") == 0; });
        ctl.solver().setContacts(ctl.egressContacts);

        if(rgc != ctl.egressContacts.end())
        {
          ctl.egressContacts.erase(rgc);
          ctl.solver().setContacts(ctl.egressContacts);
        }
        else
          LOG_INFO("OOPSIE OOPS")
        //tasks::qp::ContactId cId = rgc->contactId(ctl.robots().robots);
        //Eigen::MatrixXd dof(6,6);
        //dof.setIdentity();
        //dof(5, 5) = 0;
        //auto constr = dynamic_cast<tasks::qp::ContactConstr*>(ctl.contactConstraint.contactConstr.get());
        //if(constr == 0)
        //  LOG_INFO("NOPE NOPE")
        //else
        //  constr->addDofContact(cId, dof);

        ctl.solver().addTask(ctl.efTask);

        started = true;
        LOG_INFO("Taking right gripper out")
        return false;
      }
      else
      {
        if(!done_rotating)
        {
          if(ctl.efTask->orientationTask->eval().norm() < 1e-2
              && ctl.efTask->orientationTask->speed().norm() < 1e-4)
          {
            if(wiggles_ > maxWiggles_)
              done_rotating = true;
            else
              ++wiggles_;
            sva::PTransformd lift(Eigen::Vector3d(0, 0, dist_));
            unsigned int rgindex = ctl.robot().bodyIndexByName("RARM_LINK6");
            ctl.efTask->positionTask->position((lift*ctl.robot().mbc().bodyPosW[rgindex]).translation());
            Eigen::Matrix3d r;
            if(wiggles_ % 2 == 1)
              r = (sva::PTransformd(birot).inv()*ctl.robot().mbc().bodyPosW[rgindex]).rotation();
            else
              r = (sva::PTransformd(birot)*ctl.robot().mbc().bodyPosW[rgindex]).rotation();
            ctl.efTask->orientationTask->orientation(r);
          }
          return false;
        }
        if(!done_removing)
        {
          if(ctl.efTask->positionTask->eval().norm() < 1e-2
              && ctl.efTask->positionTask->speed().norm() < 1e-4)
          {
            done_removing = true;
            ctl.solver().setContacts(ctl.egressContacts);
            auto q = ctl.robot().mbc().q;
            //int chest_i = ctl.robot().jointIndexByName("CHEST_JOINT0");
            //q[chest_i][0] = 0;
            ctl.postureTask->posture(q);
            ctl.solver().removeTask(ctl.efTask);
            LOG_INFO("Phase finished, can transit")
          }
          return false;
        }
       else
          return false;
      }
    }

  private:
    bool started;
    bool done_rotating;
    bool done_removing;
    int wiggles_;
    int maxWiggles_;
    double dist_;
    Eigen::Matrix3d rot;
    Eigen::Matrix3d birot;
};

struct EgressMRStandupPhase : public EgressMRPhaseExecution
{
  public:
    EgressMRStandupPhase(Eigen::Vector3d offset)
      : started(false),
        done_standup(false),
        altitude_(offset),
        otherContacts()
    {
      LOG_INFO("In egress standup phase")
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(!started)
      {
        LOG_INFO("Starting standup")
        ctl.solver().removeTask(ctl.efTask);
        ctl.efTask.reset(new mc_tasks::EndEffectorTask("BODY", ctl.robots(), 0));
        unsigned int lfindex = ctl.robot().bodyIndexByName("LLEG_LINK5");
        unsigned int rfindex = ctl.robot().bodyIndexByName("RLEG_LINK5");
        const auto & lfPose = ctl.robot().mbc().bodyPosW[lfindex];
        const auto & rfPose = ctl.robot().mbc().bodyPosW[rfindex];
        Eigen::Vector3d bodyTarget = (lfPose.translation() + rfPose.translation())/2 + altitude_;
        bodyTarget(2) += 0.76;
        ctl.efTask->positionTask->position(bodyTarget);
        ctl.efTask->orientationTask->orientation(lfPose.rotation());
        ctl.solver().addTask(ctl.efTask);
        otherContacts.push_back(ctl.egressContacts.at(1));
        otherContacts.push_back(ctl.egressContacts.at(2));
        otherContacts.push_back(ctl.egressContacts.at(3));
        for(auto c: otherContacts)
        {
          LOG_INFO(c.r1Surface()->name() << " / " << c.r2Surface()->name())
        }
        ctl.solver().setContacts(otherContacts);

        auto p = ctl.postureTask->posture();
        unsigned int shoulder_i = ctl.robot().jointIndexByName("LARM_JOINT0");
        p[shoulder_i][0] = M_PI/2;
        ctl.postureTask->posture(p);

        timeoutIter = 0;
        started = true;
      }
      else if(!done_standup)
      {
        ++timeoutIter;
        if((ctl.efTask->positionTask->eval().norm() < 1e-1 && ctl.efTask->orientationTask->speed().norm() < 1e-4 && ctl.efTask->positionTask->speed().norm() < 1e-4) || timeoutIter > 5*500)
        {
          ctl.postureTask->posture(ctl.robot().mbc().q);
          ctl.solver().removeTask(ctl.efTask);
          ctl.egressContacts.erase(ctl.egressContacts.begin());
          done_standup = true;
          LOG_INFO("Finished standup")
          LOG_INFO("Can transit")
          //return true;
        }
      }
      return false;
    }

  private:
    bool started;
    bool done_standup;
    Eigen::Vector3d altitude_;
    std::vector<mc_rbdyn::Contact> otherContacts;
};

struct EgressMoveComSurfPhase : public EgressMRPhaseExecution
{
  public:
    EgressMoveComSurfPhase(std::string surfName, double altitude, bool auto_transit = false)
      : started(false),
        done_com(false),
        iter_(0),
        altitude_(altitude),
        surfName_(surfName),
        auto_transit(auto_transit)
    {
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(!started)
      {
        LOG_INFO("Moving com over " << surfName_)
        const auto & target = ctl.robot().surface(surfName_);
        Eigen::Vector3d pos = target.X_0_s(ctl.robot()).translation();
        pos(2) = pos(2) + 0.76 + altitude_;

        ctl.comTask->com(pos);
        ctl.solver().addTask(ctl.comTask);
        started = true;
        return false;
      }
      else
      {
        if(!done_com)
        {
          ++iter_;
          if((ctl.comTask->eval().norm() < 1e-2
              && ctl.comTask->speed().norm() < 1e-3)
              || iter_ > 10*500)
          {
            done_com = true;
            ctl.postureTask->posture(ctl.robot().mbc().q);
            //ctl.solver().removeTask(ctl.comTask);
            LOG_INFO("Phase finished, can transit")
            //return true;
            return auto_transit;
          }
          return false;
        }
        else
          return false;
      }
    }

  private:
    bool started;
    bool done_com;
    int iter_;
    double altitude_;
    std::string surfName_;
    bool auto_transit;
};

struct EgressCenterComPhase : public EgressMRPhaseExecution
{
  public:
    EgressCenterComPhase(double altitude)
      : started(false),
        done_com(false),
        iter_(0),
        altitude_(altitude)
    {
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(!started)
      {
        LOG_INFO("Centering com ")
        const auto & rfs = ctl.robot().surface("RFullSole");
        const auto & lfs = ctl.robot().surface("LFullSole");
        Eigen::Vector3d pos = (rfs.X_0_s(ctl.robot()).translation()
                               +lfs.X_0_s(ctl.robot()).translation())/2;
        pos(2) = pos(2) + 0.76 + altitude_;

        ctl.comTask->com(pos);
        ctl.solver().addTask(ctl.comTask);
        started = true;
        return false;
      }
      else
      {
        if(!done_com)
        {
          ++iter_;
          if((ctl.comTask->eval().norm() < 1e-2
              && ctl.comTask->speed().norm() < 1e-3)
              || iter_ > 10*500)
          {
            done_com = true;
            ctl.postureTask->posture(ctl.robot().mbc().q);
            //ctl.solver().removeTask(ctl.comTask);
            LOG_INFO("Centered com, error "
                      << ctl.comTask->eval().transpose())
            //return true;
          }
          return false;
        }
        else
        {
          return false;
        }
      }
    }

  private:
    bool started;
    bool done_com;
    int iter_;
    double altitude_;
};

struct EgressMoveComForcePhase : public EgressMRPhaseExecution
{
  public:
    EgressMoveComForcePhase(std::string surfName, double altitude, double max_move)
      : started(false),
        done_com(false),
        iter_(0),
        altitude_(altitude),
        comDist_(0),
        maxMove_(max_move),
        curCom(0, 0, 0),
        startPos_(0, 0, 0),
        surfName_(surfName)
    {
      if(surfName_.compare("LFullSole") == 0)
      {
        otherSurf_ = "RFullSole";
        bodyName_ = "RLEG_LINK5";
      }
      else if(surfName_.compare("RFullSole") == 0)
      {
        otherSurf_ = "LFullSole";
        bodyName_ = "LLEG_LINK5";
      }
    }

    virtual void updateCom(MCEgressMRQPController & ctl)
    {
       curCom = rbd::computeCoM(ctl.robot().mb(),
                                ctl.robot().mbc());
    }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(!started)
      {
        LOG_INFO("Moving com over " << surfName_
                  << " && lifting " << otherSurf_)
        const auto & target = ctl.robot().surface(surfName_);
        Eigen::Vector3d pos = target.X_0_s(ctl.robot()).translation();
        pos(2) += 0.76 + altitude_;

        updateCom(ctl);
        comDist_ = (pos-curCom).norm();

        ctl.comTask->com(pos);
        ctl.comTask->stiffness(0.5);
        ctl.solver().addTask(ctl.comTask);

        ctl.solver().removeTask(ctl.efTask);
        ctl.efTask.reset(new mc_tasks::EndEffectorTask(bodyName_,
                                                       ctl.robots(),
                                                       ctl.robots().robotIndex(),
                                                       2.0, 1000));

        ctl.solver().setContacts(ctl.egressContacts);

        auto lfc = std::find_if(ctl.egressContacts.begin(),
                                ctl.egressContacts.end(),
                                [&](const mc_rbdyn::Contact & c) -> bool { return c.r1Surface()->name().compare(otherSurf_) == 0; });
        tasks::qp::ContactId cId = lfc->contactId(ctl.robots());
        Eigen::MatrixXd dof(6,6);
        dof.setIdentity();
        dof(5, 5) = 0;
        auto constr = dynamic_cast<tasks::qp::ContactConstr*>(ctl.contactConstraint.contactConstr.get());
        if (constr == 0)
        {
          LOG_INFO("NOPE NOPE")
        }
        else
        {
          constr->resetDofContacts();
          LOG_INFO("Added a dof to " << lfc->r1Surface()->name() << " / "
            << lfc->r2Surface()->name())
            constr->addDofContact(cId, dof);
        }

        unsigned int bodyIndex = ctl.robot().bodyIndexByName(bodyName_);
        startPos_ = ctl.robot().mbc().bodyPosW[bodyIndex].translation();
        ctl.solver().addTask(ctl.efTask);
        started = true;
        LOG_INFO("Going for it with a max displacement of " << maxMove_)
        return false;
      }
      else
      {
        if(!done_com)
        {
          ++iter_;
          if((ctl.comTask->eval().norm() < 1e-2
              && ctl.comTask->speed().norm() < 1e-3)
              || iter_ > 10*500)
          {
            done_com = true;
            ctl.postureTask->posture(ctl.robot().mbc().q);
            auto constr = dynamic_cast<tasks::qp::ContactConstr*>(ctl.contactConstraint.contactConstr.get());
            if (constr == 0)
            {
              LOG_INFO("NOPE NOPE")
            }
            else
            {
              constr->resetDofContacts();
            }
            //ctl.solver().removeTask(ctl.comTask);
            LOG_INFO("Phase finished, can transit")
            //return true;
          }
          else
          {
            updateCom(ctl);
            double height = maxMove_*(1-((ctl.comTask->com()-curCom).norm()/comDist_));
            Eigen::Vector3d target(startPos_(0), startPos_(1), startPos_(2)+height);
            ctl.efTask->positionTask->position(target);
            return false;
          }
        return false;
        }
        else
          return false;
      }
    }

  private:
    bool started;
    bool done_com;
    int iter_;
    double altitude_;
    double comDist_;
    double maxMove_;
    Eigen::Vector3d curCom;
    Eigen::Vector3d startPos_;
    std::string surfName_;
    std::string otherSurf_;
    std::string bodyName_;
};

struct EgressReplaceRightHandPhase : public EgressMRPhaseExecution
{
  public:
    EgressReplaceRightHandPhase()
      : started(false),
        done_moving(false),
        done_posturing(false),
        nrIter_(0)
  {
  }

    virtual bool run(MCEgressMRQPController & ctl) override
    {
      if(!started)
      {
        LOG_INFO("Replacing hand")
        started = true;
        ctl.solver().removeTask(ctl.efTask);

        ctl.efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6",
                                                       ctl.robots(),
                                                       ctl.robots().robotIndex(),
                                                       0.25));
        unsigned int headIndex = ctl.robot().bodyIndexByName("HEAD_LINK1");
        sva::PTransformd move(Eigen::Vector3d(0.5, 0., 0.5));

        ctl.efTask->set_ef_pose(move*ctl.robot().mbc().bodyPosW[headIndex]);
        ctl.solver().addTask(ctl.efTask);
        return false;
      }
      else
      {
        if(!done_moving)
        {
          ++nrIter_;
          if((ctl.efTask->eval().norm() < 1e-2
              && ctl.efTask->speed().norm() < 1e-3)
              || nrIter_ > 10*500)
          {
            done_moving = true;
            ctl.solver().removeTask(ctl.efTask);
            auto stance = ctl.robot().stance();
            auto p = ctl.postureTask->posture();
            for(auto qi : stance)
            {
              p[ctl.robot().jointIndexByName(qi.first)] = qi.second;
            }
            ctl.postureTask->posture(p);
            nrIter_ = 0;
          }
          return false;
        }
        else if(!done_posturing)
        {
          ++nrIter_;
          auto paramNorm = [](const std::vector<std::vector<double>> & v)
          {
            double d = 0;
            for(const auto & vv : v) {
              for(const auto & vvi : vv) {
                d += vvi*vvi;
              }
            }
            return d;
          };
          if(paramNorm(ctl.robots().mbcs()[0].alphaD) < 1e-3
              || nrIter_ > 10*500)
          {
            done_posturing = true;
            LOG_INFO("Phase done")
          }
          return false;
        }
      return false;
      }
    }

  private:
    bool started;
    bool done_moving;
    bool done_posturing;
    int nrIter_;
};

}
