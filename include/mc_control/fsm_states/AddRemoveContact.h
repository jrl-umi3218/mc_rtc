#pragma once

#include <mc_control/mc_fsm_state.h>

namespace mc_control
{

struct AddRemoveContactStateImpl;

/** Implements a state that is able to remove or add a contact
 *
 * Configuration options:
 * - type (req.): One of ["addContact", "removeContact", "compliance"]
 * - contact (req.): Contact to be removed or added
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
