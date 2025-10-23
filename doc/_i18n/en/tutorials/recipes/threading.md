`mc_rtc` controllers are typically running within a real-time thread with strict timing requirements, that is `MCGlobalController::run()` is expected to consistently provide results faster than the computation `Timestep` (typically `2ms` or `5ms`). We typically refer to computation runs that were slower than this timestep as a `missed iteration`, which can have strong implications when running on real robots' hardware, depending on how the low-level hardware handles it. Common symptoms of missed iterations are: jitter, joint noises, jerky motion, etc. When an iteration is missed, the command sent to the robot remains the same as the one from the previous iteration(s):

- **In position control**: this causes the motor to be asked to abruptly stop at the current position, then to abruptly start again once a new command is received (causing jitter and/or actuator damage)
- **In velocity control**: we have two choices: either set the velocity to 0 which amounts to the same as the position control case, or keep the previously computed velocity (in that case the robot keeps moving at the latest known velocity until a new command is received).
- **In torque control**: we obviousy cannot set the torque to zero, and the only reasonable choice is to keep the previous torque.

Furthermore, many algorithms assume that sensor data is received at a constant rate defined solely by the `timestep`.

In any case, it is clear than missing iterations, especially multiple ones in a row is a bad idea and should be avoided at all cost. Let's look at common reasons causing missed iterations:
- **Controller too slow**: it should be obvious that on average your controller should compute much faster than the control `timestep`. Aim for an average computation time less than 75% of `timestep`
- **Memory allocations**: memory allocations cause cause large spikes in computation time. Allocating memory within a controller is not strictly-speaking real-time: one needs to ask the OS to allocate memory for us.
  *Recommendation*: Avoid it whenever possible, prefer doing large memory allocations ahead of time.
- **I/O operations**: Writing/reading to/from disk (especially without SSD), network operations, writing to the terminal are all non-rt operations.
  *Recommendation*: Such operations within the real-time control loop should be avoided, or performed by a thread outside of the control loop.
- **QP**: Adding Tasks/Constraints to the QP can be costly and cause a spike in computation times. The cost here is two-fold: the problem matrices need to be recreated to account for the new problem size, and the solver has to find a new solution that accounts for the modified problem, which cannot always be warm-started.
  *Recommendation*: It is unavoidable to have to add/remove tasks and constraints, so this cost cannot be fully avoided. However prefer adding tasks/constraints at the least critical moments of your control loop. For instance whenever possible avoid adding constraints during high-speed motions.
- **Algorithms that are too slow**: Some algorithms are unforunately too slow to be computed within a fraction of the control timestep (typically: vision, model-predictive control, etc). In which case, they need to be threaded to not slow down the control loop.

As is obvious from the examples above, when writing complex controllers, some form of threading is often unavoidable. Unfortunately this greatly complexifies the code and is an aspect that is often misunderstood and mismanaged, which can easily lead to bugs, oftentimes serious and unpredicatbles (undefined behaviour). In the remainder of this tutorial we will:
- Provide a quick primer on threading and its pitfalls
- Examine threading in the context of `mc_rtc`: what is safe/unsafe?
- Describe some helpers provided by `mc_rtc` to make your life easier.


# A primer on threading

A **thread** is a separate flow of execution within your program. In the context of `mc_rtc` controllers, threads are often used to offload operations that are too slow or unpredictable for the real-time control loop, such as logging, file I/O, or vision processing. This allows the main control loop to maintain its strict timing requirements while slower tasks run in parallel.

However, using threads introduces **data synchronisation challenges**. If multiple threads need to access or modify shared data (for example, sensor readings or control commands), you must ensure that this access is properly coordinated. Failing to do so can result in race conditions, where the outcome depends on the precise timing of thread execution, potentially leading to inconsistent or corrupted data. Synchronisation primitives like mutexes or locks can help, but they must be used carefully: holding a lock in the real-time thread can cause delays and missed iterations if another thread is holding the lock.

Finally, **creating a thread has a cost**. Starting a new thread requires the operating system to allocate resources and manage scheduling, which can take a significant amount of time compared to the control loop’s timestep. Typically this is in the order of dozen of microseconds, but is somewhat unpredicatble as it relies on the kernel. For this reason, threads should be created ahead of time and reused, rather than created and destroyed within the control loop. A convenient alternative is to use thread pools, which manage a set of pre-created threads that can be reused for multiple tasks.

## Data synchronization: general overview

When multiple threads access shared data, you must ensure that only one thread modifies the data at a time, or that reads and writes do not interfere with each other. This is called **data synchronization**. Without proper synchronization, you risk **race conditions**—subtle bugs where the program’s behavior depends on the unpredictable timing of threads. In real-time control, this can lead to corrupted sensor data, invalid commands, or even unsafe robot behavior.

The most common synchronization tool in C++11 is the `std::mutex`. A mutex (mutual exclusion) allows only one thread to access a critical section of code at a time. To use a mutex, you lock it before accessing shared data and unlock it afterward. C++11 provides `std::lock_guard` to automatically manage locking and unlocking. **Warning**: mutexes are not magic, they need to be locked on both sides, in the thread that read/writes the data it protects as well as in the thread that read/writes it (controller, plugin, etc).

### Example: Protecting shared data with a mutex

Suppose you have a shared variable `shared_value` that is updated by a background thread and read by the real-time control thread:

**1. Using a more complex shared object with a mutex**

Suppose the shared data is a struct representing some sensor data:

````cpp
#include <mutex>
#include <thread>
#include <iostream>

struct SensorData
{
  double position;
  double velocity;
};

SensorData shared_sensor_data{0.0, 0.0};
std::mutex mtx;

void background_thread()
{
  for(int i = 0; i < 10; ++i)
  {
    {
      std::lock_guard<std::mutex> lock(mtx);
      shared_sensor_data.position += 1.0;
      shared_sensor_data.velocity += 0.5;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void control_thread()
{
  for(int i = 0; i < 10; ++i)
  {
    SensorData local_copy;
    {
      std::lock_guard<std::mutex> lock(mtx);
      local_copy = shared_sensor_data;
    }
    std::cout << "Position: " << local_copy.position
              << ", Velocity: " << local_copy.velocity << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
````

**2. Using atomic types for simple data**

For simple types like integers or booleans, you can use `std::atomic` for lock-free synchronization:

````cpp
#include <atomic>
#include <thread>
#include <iostream>

std::atomic<int> shared_counter{0};

void background_thread()
{
  for(int i = 0; i < 10; ++i)
  {
    ++shared_counter; // atomic increment
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void control_thread()
{
  for(int i = 0; i < 10; ++i)
  {
    int value = shared_counter.load();
    std::cout << "Counter: " << value << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
````

**Note:**
Atomic types are only suitable for simple data (like integers, booleans, etc). For complex objects, use mutexes or specialized lock-free data structures.

## Data synchronization in mc_rtc

It is important to note that data structures in `mc_rtc` are generally **not thread-safe**. This means that if you access or modify `mc_rtc` data (like `MCGlobalController`, `Robot`, `Task`, etc.) from multiple threads, you must ensure proper synchronization to avoid running into undefined behaviour.

Here is an unsafe example where a separate thread reads robot data while the main thread modifies it:

### Unsafe example

```cpp
#include <mc_rbdyn/Robot.h>
#include <thread>
#include <iostream>

void unsafe_read(mc_rbdyn::Robot * robot)
{
  // This function reads robot data in a separate thread
  for(int i = 0; i < 100; ++i)
  {
    // Potentially unsafe: another thread might modify robot->posW() at the same time
    // The result here is undefined behavior, it might appear to work, crash, or produce garbage values
    std::cout << "Robot position: " << robot->posW().translation().transpose() << std::endl;
  }
}

void unsafe_example(mc_rbdyn::Robot * robot)
{
  // Start a thread that reads robot data
  std::thread t(unsafe_read, robot);

  // Meanwhile, the main thread modifies the robot data
  for(int i = 0; i < 100; ++i)
  {
    // Potentially unsafe: this modifies the robot's position
    robot->posW(sva::PTransformd{sva::RotZ(i * mc_rtc::constants::PI), Eigen::Vector3d{i/10., 0, 0}});
  }

  t.join();
}
```

**Warning:**
This example is unsafe because both threads access and modify the same `mc_rbdyn::Robot` object without any synchronization (like mutexes). This can cause data races and undefined behavior.

### Safe example

Now here’s a **safe version** using a `std::mutex` to synchronize access to the `mc_rbdyn::Robot` object. This prevents concurrent reads/writes and avoids data races.

```cpp
#include <mc_rbdyn/Robot.h>
#include <thread>
#include <mutex>
#include <iostream>

std::mutex robot_mutex;

void safe_read(mc_rbdyn::Robot * robot)
{
  for(int i = 0; i < 100; ++i)
  {
    std::lock_guard<std::mutex> lock(robot_mutex);
    std::cout << "Robot position: " << robot->posW().translation().transpose() << std::endl;
  }
}

void safe_example(mc_rbdyn::Robot * robot)
{
  std::thread t(safe_read, robot);

  for(int i = 0; i < 100; ++i)
  {
    std::lock_guard<std::mutex> lock(robot_mutex);
    robot->posW(sva::PTransformd{sva::RotZ(i * mc_rtc::constants::PI), Eigen::Vector3d{i/10., 0, 0}});
  }

  t.join();
}
```

Now, all accesses to the `robot` object are protected by a mutex, ensuring thread safety.

## Deadlocks with mutexes

A **deadlock** occurs when two or more threads are each waiting for the other to release a resource (such as a mutex), causing all threads involved to wait forever. This typically happens when multiple mutexes are locked in different orders by different threads.

The following example demonstrates how improper use of mutexes can lead to a deadlock:

```cpp
#include <mutex>
#include <thread>
#include <iostream>

std::mutex mutexA;
std::mutex mutexB;

void thread1()
{
  std::lock_guard<std::mutex> lockA(mutexA);
  // Simulate work
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::lock_guard<std::mutex> lockB(mutexB); // Waits for mutexB
  std::cout << "Thread 1 acquired both mutexes\n";
}

void thread2()
{
  std::lock_guard<std::mutex> lockB(mutexB);
  // Simulate work
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::lock_guard<std::mutex> lockA(mutexA); // Waits for mutexA
  std::cout << "Thread 2 acquired both mutexes\n";
}

void deadlock_example()
{
  std::thread t1(thread1);
  std::thread t2(thread2);
  t1.join();
  t2.join();
}
```


# mc_rtc threading utilities

To help with common cases encountered while working within the framework, `mc_rtc` provides some utilities to help you manage threading and data synchronization:
- `mc_rtc::threading::AsyncJob`: Provides a convenient way to run jobs asynchronously in a separate thread. This creates and manages and `std::async` job.

## AsyncJob

The `mc_rtc::threading::AsyncJob` class provides a convenient way to run jobs asynchronously in a separate thread. It uses `std::async` under the hood to manage the thread and provides methods to check if the job is done, retrieve the result, and handle exceptions.

**Advantages:**
- Easy to use
- Clear separation of inputs/outputs

**Drawbacks:**
- Creates a separate thread for each job, which can be costly if jobs are frequent and short-lived.


### How to use `AsyncJob`

The `AsyncJob` class is a CRTP (Curiously Recurring Template Pattern) base class for running asynchronous computations in `mc_rtc`. It manages job state, result retrieval, and optional integration with logging and GUI.

**Steps to use:**

1. **Create a MyInput class**
   This should contain a copy of all data needed to run the job.

2. **Create a MyResult class**
   This should contain a copy of all data produced by the job.

3. **Define your job class**
   Inherit from `mc_rtc::threading::MakeAsyncJob<YourJob, InputType, ResultType>`.
   Implement the required `ResultType computeJob()` method.

4. **(Optional) Add logging and GUI**
   Implement `void addToLoggerImpl()` and/or `void addToGUIImpl()` for custom log/GUI elements.

5. **Create and use your job**
   - Set the input (when not running).
   - Call `startAsync()` to launch the computation.
   - Regularly call `checkResult()` in your control loop to update state and retrieve results.
   - Access the result via `lastResult()`.

**Notes:**
- Only modify the input when the job is not running (`running() == false`).
- Always call `checkResult()` in your control loop to handle results and bookkeeping.
- Logger and GUI entries are automatically removed when the job is destroyed.

**Basic Example:**

```cpp
// MyAsyncJob.h

struct MyInput
{
  double data; // Input value for the job
};

struct MyResult
{
  double value; // Computed result
};

struct MyAsyncJob : public mc_rtc::threading::MakeAsyncJob<MyAsyncJob, MyInput, MyResult>
{
  MyResult computeJob()
  {
    MyResult res;
    res.value = input_.data * 2;
    return res;
  }

  void addToLoggerImpl()
  {
    logger_->addLogEntry(loggerPrefix_ + "_my_job_value", this, [this]() { return lastResult_->value; });
  }

  void addToGUIImpl()
  {
    gui_->addElement(this, guiCategory_, mc_rtc::gui::Label("Result", [this]() { return lastResult_->value; }));
  }
};
```

**Example FSM State using MyAsyncJob:**

```cpp
// SimpleAsyncState.h
#include <mc_control/fsm/State.h>
#include <mc_rtc/threading/AsyncJob.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>
#include "MyAsyncJob.h"

// FSM State using the simple AsyncJob, restarting after each result
struct SimpleAsyncState : public mc_control::fsm::State
{
  MyAsyncJob job_; // Creates the job class (does not start a job)
  int counter_ = 0;
  int maxIterations_ = 5; // Stop after 5 results

  void configure(const mc_rtc::Configuration & config) override
  {
    // setup initial input for the async job
    config("input", job_.input().data);
    config("maxIterations", maxIterations_);
  }

  void start(mc_control::fsm::Controller & ctl) override
  {
    // Start the first async job. The result will be available later in run() using job_.checkResult()
    job_.startAsync();
    job_.addToLogger(ctl.logger(), name());
    job_.addToGUI(ctl.gui(), {name()});
    counter_ = 0;
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    if(job_.checkResult())
    { // The previous async job completed, we can retrieve the result
      const auto & result = *job_.lastResult();
      mc_rtc::log::info("[{}] Async result {}: {}", name(), counter_, result.value);

      counter_++;

      // The job is no longer running, it is safe to update the input for the next job
      job_.input().data += 1.0;

      if(counter_ < maxIterations_)
      {
        job_.startAsync(); // Restart the async job
        return false;      // Continue running
      }
      else
      {
        output("OK");      // End state after maxIterations_
        return true;
      }
    }
    return false;
  }

  void teardown(mc_control::fsm::Controller & ctl) override
  {
    // Cleanup handled by AsyncJob destructor
  }
};
```
