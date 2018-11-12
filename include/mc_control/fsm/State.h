#pragma once

#include <mc_control/fsm/api.h>
#include <mc_control/fsm/states/api.h>
#include <mc_rtc/Configuration.h>

namespace mc_control
{

namespace fsm
{

struct MC_CONTROL_FSM_DLLAPI Controller;

/** \class State
 *
 * A state of an FSM.
 *
 * A state implementation should create, add and remove the tasks/constraints
 * it needs, with the following exceptions:
 * - Contacts are handled at the global level, the state should go through the
 *   addContact/removeContact methods of FSMController
 * - Collision constraints are handled at the global level, the state should go
 *   through the addCollisons/removeCollisions methods of Controller
 * - Kinematics/Dynamics constraints are handled at the global level
 * - Posture tasks are handled at the global level, if a state removes a
 *   posture task from the solver, it should put it back afterwards
 *
 * Notably, a state should not keep constraints or tasks it created actives
 * after the state finished its execution.
 *
 * Every methods can be called in a real-time context. You should be aware of
 * that, especially if you plan to have threads in your state (e.g. monitoring
 * a ROS topic, waiting for data on the network...) then you should be
 * especially careful when your state is destroyed.
 *
 * The life-span of a state is:
 * - the state is constructed and init is called (iteration 0)
 * - run(...) is called until it returns true (iteration 1 to N)
 * - teardown(...) is called and the state is destructed (iteration N, after
 *   run last call)
 *
 * The state might be interrupted (e.g. emergency behaviour triggered) in which
 * case, stopped(...) will be called. The default implementation does nothing
 * as teardown(...) is called anyway.
 *
 */
struct MC_CONTROL_FSM_DLLAPI State
{
  virtual ~State() {}

  /** Common implementation, handles the following options:
   *
   * - AddContacts/RemoveContacts: add and remove contacts during the state's
   *   start
   * - AddContactsAfter/RemoveContactsAfter: add and remove contacts during the
   * - AddCollisions/RemoveCollisions: add and remove collisions during the state's
   *   start
   * - AddCollisionsAfter/RemoveCollisionsAfter: add and remove collisions during the
   *   state's teardown
   * - RemovePostureTask: if true, remove the robot posture task at the state's
   *   start
   */
  void configure_(const mc_rtc::Configuration & config);

  /** Common implementation, takes care of common options */
  void start_(Controller & ctl);

  /** Common implementation, takes care of common options */
  void teardown_(Controller & ctl);

  /** Called every iteration until it returns true */
  virtual bool run(Controller & ctl) = 0;

  /** Called if the state is interrupted */
  virtual void stop(Controller &) {}

  /** Handle read service call */
  virtual bool read_msg(std::string &)
  {
    return false;
  }

  /** Handle read/write service call */
  virtual bool read_write_msg(std::string &, std::string &)
  {
    return false;
  }

  /** Returns the output of the state, should only be consulted once run has
   * returned true */
  const std::string & output()
  {
    return output_;
  }

  /** Returns the name of the state */
  const std::string & name()
  {
    return name_;
  }

  void name(const std::string & n)
  {
    name_ = n;
  }

protected:
  /** Output setter for derived classes */
  void output(const std::string & o)
  {
    output_ = o;
  }

  /** Called to configure the state.
   *
   * This might be called multiple times.
   *
   */
  virtual void configure(const mc_rtc::Configuration & config) = 0;

  /** Called before the state starts being run
   *
   * This will be called only once with the state fully configured.
   *
   */
  virtual void start(Controller & ctl) = 0;

  /** Called right before destruction */
  virtual void teardown(Controller & ctl) = 0;

protected:
  mc_rtc::Configuration add_contacts_config_;
  mc_rtc::Configuration remove_contacts_config_;
  mc_rtc::Configuration add_contacts_after_config_;
  mc_rtc::Configuration remove_contacts_after_config_;
  mc_rtc::Configuration add_collisions_config_;
  mc_rtc::Configuration remove_collisions_config_;
  mc_rtc::Configuration add_collisions_after_config_;
  mc_rtc::Configuration remove_collisions_after_config_;
  bool remove_posture_task_ = false;

private:
  std::string name_ = "";
  std::string output_ = "";
};

using StatePtr = std::shared_ptr<State>;

} // namespace fsm

} // namespace mc_control

/* The following macros are used to simplify the required symbol exports */

#ifdef WIN32
#  define FSM_STATE_API __declspec(dllexport)
#else
#  if __GNUC__ >= 4
#    define FSM_STATE_API __attribute__((visibility("default")))
#  else
#    define FSM_STATE_API
#  endif
#endif

#define EXPORT_SINGLE_STATE(NAME, TYPE)                                   \
  extern "C"                                                              \
  {                                                                       \
    FSM_STATE_API void MC_RTC_FSM_STATE(std::vector<std::string> & names) \
    {                                                                     \
      names = {NAME};                                                     \
    }                                                                     \
    FSM_STATE_API void destroy(mc_control::fsm::State * ptr)              \
    {                                                                     \
      delete ptr;                                                         \
    }                                                                     \
    FSM_STATE_API mc_control::fsm::State * create(const std::string &)    \
    {                                                                     \
      return new TYPE();                                                  \
    }                                                                     \
  }
