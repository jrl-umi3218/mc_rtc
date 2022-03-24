/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/SimulationContactPair.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/AddRemoveContact.h>
#include <mc_rbdyn/Contact.h>
#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_control
{

namespace fsm
{

namespace
{
template<typename T>
struct always_false : public std::false_type
{
};
} // namespace

template<typename T>
struct AddRemoveContactStateImplHelper
{
  using task_t = T;
  using task_ptr = std::shared_ptr<T>;
  static void make_run(AddRemoveContactStateImpl & impl,
                       Controller & ctl,
                       mc_rbdyn::Contact & contact,
                       mc_rtc::Configuration & config);

  static void make_run_impl(AddRemoveContactStateImpl &, Controller &, mc_rbdyn::Contact &, mc_rtc::Configuration &)
  {
    static_assert(always_false<T>::value, "AddRemoveContactStateImplHelper not implemented for this type");
  }
};

template<>
void AddRemoveContactStateImplHelper<mc_tasks::RemoveContactTask>::make_run_impl(AddRemoveContactStateImpl & impl,
                                                                                 Controller & ctl,
                                                                                 mc_rbdyn::Contact & contact,
                                                                                 mc_rtc::Configuration & config);

template<>
void AddRemoveContactStateImplHelper<mc_tasks::AddContactTask>::make_run_impl(AddRemoveContactStateImpl & impl,
                                                                              Controller & ctl,
                                                                              mc_rbdyn::Contact & contact,
                                                                              mc_rtc::Configuration & config);

template<>
void AddRemoveContactStateImplHelper<mc_tasks::force::ComplianceTask>::make_run_impl(AddRemoveContactStateImpl & impl,
                                                                                     Controller & ctl,
                                                                                     mc_rbdyn::Contact & contact,
                                                                                     mc_rtc::Configuration & config);

struct AddRemoveContactStateImpl
{
  std::shared_ptr<mc_tasks::MetaTask> task_ = nullptr;
  std::shared_ptr<mc_tasks::CoMTask> com_task_ = nullptr;
  bool useCoM_ = true;
  std::function<bool(AddRemoveContactStateImpl &, Controller &)> run_ = [](AddRemoveContactStateImpl &, Controller &) {
    return true;
  };
  void start(Controller & ctl, mc_rtc::Configuration & config_)
  {
    auto contact = mc_rbdyn::Contact::load(ctl.robots(), config_("contact"));
    config_("useCoM", useCoM_);
    if(useCoM_)
    {
      com_task_ = std::make_shared<mc_tasks::CoMTask>(ctl.robots(), contact.r1Index());
      com_task_->reset();
      if(config_.has("com"))
      {
        com_task_->load(ctl.solver(), config_("com"));
      }
    }
    std::string type = config_("type");
    bool removeContact = (type == "removeContact");
    bool isCompliant = (type == "compliance");
    if(isCompliant)
    {
      std::string body = contact.r1Surface()->bodyName();
      if(ctl.solver().robot(contact.r1Index()).bodyHasForceSensor(body))
      {
        config_.add("robotIndex", contact.r1Index());
        config_.add("body", body);
      }
      else
      {
        mc_rtc::log::error("AddRemoveContactState configured with compliant task but surface {} is attached to body {} "
                           "which does not have a force sensor.",
                           contact.r1Surface()->name(), body);
        mc_rtc::log::warning("Defaulting to simulated contact sensor");
        config_.add("type", "addContact");
        isCompliant = false;
        if(!config_.has("stiffness"))
        {
          config_.add("stiffness", 2.0);
        }
        if(!config_.has("weight"))
        {
          config_.add("weight", 1000.0);
        }
        if(!config_.has("speed"))
        {
          config_.add("speed", 0.01);
        }
      }
    }
    bool hasContact =
        ctl.hasContact({ctl.solver().robot(contact.r1Index()).name(), ctl.solver().robot(contact.r2Index()).name(),
                        contact.r1Surface()->name(), contact.r2Surface()->name()});
    if((hasContact && removeContact) || (!hasContact && !removeContact))
    {
      if(removeContact)
      {
        AddRemoveContactStateImplHelper<mc_tasks::RemoveContactTask>::make_run(*this, ctl, contact, config_);
      }
      else
      {
        if(isCompliant)
        {
          AddRemoveContactStateImplHelper<mc_tasks::force::ComplianceTask>::make_run(*this, ctl, contact, config_);
        }
        else
        {
          AddRemoveContactStateImplHelper<mc_tasks::AddContactTask>::make_run(*this, ctl, contact, config_);
        }
      }
      std::string name = contact.r1Surface()->name() + "_" + contact.r2Surface()->name() + "_com";
      if(removeContact)
      {
        name = "RemoveContact_" + name;
      }
      else
      {
        name = "AddContact_" + name;
      }
      if(useCoM_)
      {
        com_task_->name(name);
        ctl.solver().addTask(com_task_);
      }
    }
    else
    {
      mc_rtc::log::info("AddRemoveContactState has nothing to do here");
    }
  }
};

template<typename T>
void AddRemoveContactStateImplHelper<T>::make_run(AddRemoveContactStateImpl & impl,
                                                  Controller & ctl,
                                                  mc_rbdyn::Contact & contact,
                                                  mc_rtc::Configuration & config)
{
  task_ptr t = mc_tasks::MetaTaskLoader::load<task_t>(ctl.solver(), config);
  impl.task_ = t;
  ctl.solver().addTask(impl.task_);
  make_run_impl(impl, ctl, contact, config);
}

template<>
void AddRemoveContactStateImplHelper<mc_tasks::RemoveContactTask>::make_run_impl(AddRemoveContactStateImpl & impl,
                                                                                 Controller & ctl,
                                                                                 mc_rbdyn::Contact & contact,
                                                                                 mc_rtc::Configuration & config)
{
  ctl.removeContact(Contact::from_mc_rbdyn(ctl, contact));
  double distance_ = config("distance", 0.075);
  auto robotIndex_ = contact.r1Index();
  auto bodyIndex_ = contact.r1Surface()->bodyIndex(ctl.robots().robot(contact.r1Index()));
  auto init_pos_ = ctl.robots().robot(robotIndex_).bodyPosW()[bodyIndex_].translation();
  bool reached_ = false;
  impl.run_ = [distance_, robotIndex_, bodyIndex_, init_pos_, reached_](AddRemoveContactStateImpl & impl,
                                                                        Controller & ctl) mutable {
    const auto & pos = ctl.robots().robot(robotIndex_).bodyPosW()[bodyIndex_].translation();
    auto d = (pos - init_pos_).norm();
    if(d >= distance_)
    {
      if(!reached_)
      {
        reached_ = true;
        ctl.solver().removeTask(impl.task_);
        auto t = std::static_pointer_cast<mc_tasks::RemoveContactTask>(impl.task_);
        auto w = t->weight();
        auto s = t->stiffness();
        impl.task_ = std::make_shared<mc_tasks::EndEffectorTask>(
            ctl.robots().robot(robotIndex_).mb().body(static_cast<int>(bodyIndex_)).name(), ctl.robots(), robotIndex_,
            s, w);
        ctl.solver().addTask(impl.task_);
      }
    }
    return d >= distance_;
  };
}

template<>
void AddRemoveContactStateImplHelper<mc_tasks::AddContactTask>::make_run_impl(AddRemoveContactStateImpl & impl,
                                                                              Controller & ctl,
                                                                              mc_rbdyn::Contact & contact,
                                                                              mc_rtc::Configuration & config)
{
  auto fsm_contact_ = new Contact(Contact::from_mc_rbdyn(ctl, contact));
  auto sensor_ = SimulationContactPair(contact.r1Surface(), contact.r2Surface());
  auto robotIndex_ = contact.r1Index();
  auto envIndex_ = contact.r2Index();
  auto forceThreshold_ = config("forceThreshold", std::numeric_limits<double>::infinity());
  bool forceOnly_ = config("forceOnly", false);
  size_t forceThresholdIter_ = static_cast<size_t>(config("forceThresholdIter", 3));
  bool hasForceSensor_ = ctl.robot().bodyHasForceSensor(contact.r1Surface()->bodyName());
  auto forceSensorName_ = hasForceSensor_ ? ctl.robot().bodyForceSensor(contact.r1Surface()->bodyName()).name() : "";
  size_t forceIter_ = 0;
  impl.run_ = [fsm_contact_, sensor_, robotIndex_, envIndex_, hasForceSensor_, forceThreshold_, forceSensorName_,
               forceOnly_, forceThresholdIter_,
               forceIter_](AddRemoveContactStateImpl & impl, Controller & ctl) mutable {
    if(!fsm_contact_)
    {
      return true;
    }
    auto & robot = ctl.robots().robot(robotIndex_);
    auto & env = ctl.robots().robot(envIndex_);
    auto d = sensor_.update(robot, env);
    if(hasForceSensor_ && ctl.robot().forceSensor(forceSensorName_).force().z() > forceThreshold_)
    {
      forceIter_++;
    }
    else
    {
      forceIter_ = 0;
    }
    if((!forceOnly_ && d <= 0) || forceIter_ > forceThresholdIter_)
    {
      if(fsm_contact_)
      {
        if(d <= 0)
        {
          mc_rtc::log::info("Geometric contact detected");
        }
        else
        {
          mc_rtc::log::info("Force contact detected");
        }
        ctl.addContact(*fsm_contact_);
        auto t = std::static_pointer_cast<mc_tasks::AddContactTask>(impl.task_);
        t->speed(0.0);
        delete fsm_contact_;
        fsm_contact_ = nullptr;
      }
      return true;
    }
    return false;
  };
}

template<>
void AddRemoveContactStateImplHelper<mc_tasks::force::ComplianceTask>::make_run_impl(AddRemoveContactStateImpl & impl,
                                                                                     Controller & ctl,
                                                                                     mc_rbdyn::Contact & contact,
                                                                                     mc_rtc::Configuration & config)
{
  auto fsm_contact_ = new Contact(Contact::from_mc_rbdyn(ctl, contact));
  double vel_thresh_ = config("velocity", 1e-4);
  impl.run_ = [fsm_contact_, vel_thresh_](AddRemoveContactStateImpl & impl, Controller & ctl) mutable {
    auto t = std::static_pointer_cast<mc_tasks::force::ComplianceTask>(impl.task_);
    if(t->speed().norm() < vel_thresh_ && t->eval().norm() < t->getTargetWrench().vector().norm() / 2 && fsm_contact_)
    {
      ctl.addContact(*fsm_contact_);
      delete fsm_contact_;
      fsm_contact_ = nullptr;
      return true;
    }
    return false;
  };
}

AddRemoveContactState::AddRemoveContactState() : impl_(new AddRemoveContactStateImpl()) {}

AddRemoveContactState::~AddRemoveContactState() {}

void AddRemoveContactState::start(Controller & ctl)
{
  impl_->start(ctl, config_);
}

bool AddRemoveContactState::run(Controller & ctl)
{
  if(impl_->run_(*impl_, ctl))
  {
    output("OK");
    return true;
  }
  return false;
}

void AddRemoveContactState::teardown(Controller & ctl)
{
  if(impl_->task_)
  {
    ctl.solver().removeTask(impl_->task_);
    if(impl_->useCoM_)
    {
      ctl.solver().removeTask(impl_->com_task_);
    }
  }
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("AddRemoveContact", mc_control::fsm::AddRemoveContactState)
