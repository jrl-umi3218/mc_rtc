#pragma once

#include <mc_control/fsm/State.h>

namespace mc_control
{

namespace fsm
{

struct MC_CONTROL_FSM_STATE_DLLAPI AddRemoveContactStateImpl;

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

struct AddRemoveContactState : State
{
  AddRemoveContactState();

  virtual ~AddRemoveContactState();

  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller &) override;

  bool run(Controller &) override;

  void teardown(Controller &) override;

protected:
  std::unique_ptr<AddRemoveContactStateImpl> impl_;
};

} // namespace fsm

} // namespace mc_control
