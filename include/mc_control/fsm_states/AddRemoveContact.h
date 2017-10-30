#pragma once

#include <mc_control/mc_fsm_state.h>

namespace mc_control
{

struct AddRemoveContactStateImpl;

/** Implements a state that is able to remove or add a contact
 *
 * General configuration options:
 * - type (req.): one of ["addContact", "removeContact", "compliance"]
 * - contact (req.): contact to be removed or added
 *
 * Remove contact options:
 * - distance (def: 0.1): when the contact body has moved this distance away
 *   from the contact, the state is finished
 *
 * Compliant add contact options:
 * - velocity (def: 1e-4): velocity threshold for the ComplianceTask
 *
 * Other options depend on the type of task used to add/remove the contact. For
 * the compliance task, the "body" entry is overwritten based on the contact
 * value.
 *
 * If you use a compliance task to remove a contact that has no force sensor
 * attached the state will automatically fallback to using an AddContactTask.
 *
 */

struct AddRemoveContactState : mc_control::FSMState
{
  AddRemoveContactState();

  virtual ~AddRemoveContactState();

  void configure(const mc_rtc::Configuration & config) override;

  void start(FSMController&) override;

  bool run(FSMController&) override;

  void teardown(FSMController&) override;
protected:
  std::unique_ptr<AddRemoveContactStateImpl> impl_;
};

}
