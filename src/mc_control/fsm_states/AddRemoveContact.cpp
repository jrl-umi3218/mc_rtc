#include <mc_control/fsm_states/AddRemoveContact.h>

#include <mc_control/mc_fsm_controller.h>

#include <mc_rbdyn/Contact.h>

#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_control
{

namespace
{
  template<typename T>
  struct always_false : public std::false_type {};
}

template<typename T>
struct AddRemoveContactStateImplHelper
{
  static void make_run(AddRemoveContactStateImpl &)
  {
    static_assert(always_false<T>::value, "AddRemoveContactStateImplHelper not implemented for this type");
  }
};

template<>
void AddRemoveContactStateImplHelper<mc_tasks::RemoveContactTask>::make_run(AddRemoveContactStateImpl & impl)
{
  // TODO
}

template<>
void AddRemoveContactStateImplHelper<mc_tasks::AddContactTask>::make_run(AddRemoveContactStateImpl & impl)
{
  // TODO
}

template<>
void AddRemoveContactStateImplHelper<mc_tasks::ComplianceTask>::make_run(AddRemoveContactStateImpl & impl)
{
  // TODO
}

struct AddRemoveContactStateImpl
{
  mc_rtc::Configuration config_;
  std::shared_ptr<mc_tasks::MetaTask> task_;
  std::function<bool(FSMController&)> run_ = [](FSMController&) { return true; };
  void start(FSMController & ctl)
  {
    auto contact = mc_rbdyn::Contact::load(ctl.robots(), config_("contact"));
    std::string type = config_("type");
    bool removeContact = (type == "removeContact");
    bool isCompliant = (type == "compliance");
    if(isCompliant)
    {
      std::string body = contact.r1Surface()->bodyName();
      if(ctl.solver().robot(contact.r1Index()).bodyHasForceSensor(body))
      {
        config_.add("body", body);
      }
      else
      {
        LOG_ERROR("AddRemoveContactState configured with compliant task but surface " << contact.r1Surface()->name() << " is attached to body " << body << " which does not have a force sensor.")
        LOG_WARNING("Defaulting to simulated contact sensor")
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
    bool hasContact = ctl.hasContact({ctl.solver().robot(contact.r1Index()).name(),
                                     ctl.solver().robot(contact.r2Index()).name(),
                                     contact.r1Surface()->name(),
                                     contact.r2Surface()->name()});
    if( (hasContact && removeContact) ||
        (!hasContact && !removeContact) )
    {
      if(removeContact)
      {
        AddRemoveContactStateImplHelper<mc_tasks::RemoveContactTask>::make_run(*this);
      }
      else
      {
        if(isCompliant)
        {
          AddRemoveContactStateImplHelper<mc_tasks::ComplianceTask>::make_run(*this);
        }
        else
        {
          AddRemoveContactStateImplHelper<mc_tasks::AddContactTask>::make_run(*this);
        }
      }
    }
    else
    {
      LOG_INFO("AddRemoveContactState has nothing to do here")
    }
  }
};

AddRemoveContactState::AddRemoveContactState()
: impl_(new AddRemoveContactStateImpl())
{
}

AddRemoveContactState::~AddRemoveContactState() {}

void AddRemoveContactState::configure(const mc_rtc::Configuration & config)
{
  impl_->config_.load(config);
}

void AddRemoveContactState::start(FSMController & ctl)
{
  impl_->start(ctl);
}

bool AddRemoveContactState::run(FSMController & ctl)
{
  return impl_->run_(ctl);
}

void AddRemoveContactState::teardown(FSMController & ctl)
{
  if(impl_->task_)
  {
    ctl.solver().removeTask(impl_->task_);
  }
}

}

EXPORT_SINGLE_STATE("AddRemoveContact", mc_control::AddRemoveContactState, "OK")
