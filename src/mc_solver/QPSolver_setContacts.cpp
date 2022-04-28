#include <mc_solver/QPSolver.h>

#include <mc_control/MCController.h>

namespace mc_solver
{

void QPSolver::setContacts(const std::vector<mc_rbdyn::Contact> & contacts)
{
  if(controller_)
  {
    mc_rtc::log::warning("[MC_RTC_DEPRECATED] Prefer using addContact/removeContact for contacts manipulation instead "
                         "of QPSolver::setContacts");
    controller_->clearContacts();
    for(const auto & c : contacts)
    {
      controller_->addContact(mc_control::Contact::from_mc_rbdyn(*controller_, c));
    }
  }
  else
  {
    setContacts(ControllerToken{}, contacts);
  }
}

} // namespace mc_solver
