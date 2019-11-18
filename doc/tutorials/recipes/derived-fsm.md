---
layout: tutorials
---

A common recipe to get more out of the FSM is to derive the FSM controller, this allows to:
- share data (tasks, constraints, parameters...) across states easily;
- easily implement a default behaviour (note that this can be achieved using the `Parallel` and `Meta` state).

To do so, the following steps are necessary:
- your controller must derive from `mc_control::fsm::Controller` instead of `mc_control::MCController`;
- call `mc_control::fsm::Controller::reset` first in your own reset function. Alternatively you can call `mc_control::MCController::reset` first, then do your own initialization then call `mc_control::fsm::Controller::reset`. The later is required if your first state needs data from your controller instance that will be properly initialized in the reset function;
- your library should link with `mc_control_fsm`

The states you implement for your controller must still follow the regular `State` interface, if you want to access your controller class, you should cast the `Controller` instance that is passed to you.

```cpp
bool MyState::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MyController&>(ctl_);
  // Now you can access ctl as your controller
}
```

A way to simplify the controller access above is to derive the `mc_control::fsm::State` class so that your state already access your own controller class. See the following snippet below:

```cpp
struct State : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & controller) final
  {
    start(static_cast<Controller&>(controller));
  }
  bool run(mc_control::fsm::Controller & controller) final
  {
    return run(static_cast<Controller&>(controller));
  }
  void teardown(mc_control::fsm::Controller &) final
  {
    teardown(static_cast<Controller&>(controller));
  }

  virtual void start(Controller &) = 0;
  virtual bool run(Controller &) = 0;
  virtual void teardown(Controller &) = 0;
};
```

Then simply, inherit this `State` class instead of `mc_control::fsm::State`.

You can use this technique to implement the FSM behaviour differently, see the following example:

```cpp
struct State : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & controller) override
  {
    controller_ = &static_cast<Controller&>(controller);
    start();
  }
  bool run(mc_control::fsm::Controller &) override
  {
    if (checkTransitions())
    {
      return true;
    }
    runState();
    return false;
  }
  void teardown(mc_control::fsm::Controller &) override
  {
    teardown();
  }
  Controller & controller()
  {
    return *controller_;
  }

  // frequently-used shorthand functions can be added here
  mc_rtc::Logger & logger()
  {
    return controller_->logger();
  }

  virtual bool checkTransitions() = 0;
  virtual void runState() = 0;
  virtual void start() = 0;
  virtual void teardown() = 0;

protected:
  Controller * controller_ = nullptr;
};
```

## Getting started

The `mc_rtc_new_fsm_controller` tool will let you create a new controller deriving from the FSM controller. It works similarly to the `mc_rtc_new_controller` tool.
