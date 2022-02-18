File logging is enabled/disable through the [global
configuration]({{site.baseurl}}/tutorials/introduction/configuration.html). However, a controller implementation does
not need to worry about the logging options and will go through a single
interface for all file logging operations. Each controller instance uses a
separate log and each time a controller switch happens, a new log is created.
For convenience purpose, on platforms that support symbolic links, a `latest`
log file is created pointing to the latest created log.

By default, the following information are logged by all controllers:
- Iteration time (starting from 0, increasing by the configured timestep)
- Encoder values for the main robot (stored in the robot's reference joint order)
- Free-flyer orientation (as a quaternion) and position for the main robot (controller reference)
- Joint command for the main robot (controller reference, stored in the robot's reference joint order)
- Joint torques (stored in the robot's reference joint order)
- Force sensors readings (stored as torque,force)
- Sensor free-flyer position (provided by some interface only)
- Sensor orientation (RPY)
- Sensor free-flyer linear velocity (provided by some interface only)
- Sensor angular velocity
- Sensor acceleration

### Add logging entries

Add log entries in your controller's code:

```cpp
logger().addLogEntry("entry_name", [this]() { /* compute, say x; */ return x; });
```

The provided callback will run after your controller `run()` method.

You can log most types that your controller could manipulate as-is (e.g. `sva::PTransformd` or `sva::ForceVecd`) and strings. You can also logs vectors of these types.

Normally, an `addLogEntry` call should have a corresponding `removeLogEntry` call, e.g.:

```cpp
logger().removeLogEntry("entry_name");
```

#### With a logging source

You can specify a logging source:

```cpp
// In this example, "this" is the source
logger().addLogEntry("entry_A", this, [this]() { return a; });
logger().addLogEntry("entry_B", this, [this]() { return b; });
logger().addLogEntry("entry_C", this, [this]() { return c; });

// You can also group calls with the same source
logger().addLogEntries(this,
                       "entry_A", [this]() { return a; },
                       "entry_B", [this]() { return b; },
                       "entry_C", [this]() { return c; });
```

Then all the entries can be removed at once:

```cpp
logger().removeLogEntries(this);

// This would yield the same result but you would need to keep the addLogEntry and removeLogEntry call in sync
logger().removeLogEntry("entry_A");
logger().removeLogEntry("entry_B");
logger().removeLogEntry("entry_C");
```

#### Logging members or method

As seen above, a common pattern is to load a member or method called on `this` instance, this can be done in a simple way. The main benefit of this approach is to avoid the performance caveat highlighted below.

```cpp
struct MyObject
{
  Eigen::Vector3d data_;
  sva::PTransformd something_;

  Eigen::Vector6d computeVector();

  const sva::PTransformd & referenceBody();
  void referenceBody(const std::string &);

  void addToLogger(mc_rtc::Logger & logger)
  {
    // This is the actual syntax required in C++11
    logger.addLogEntry<decltype(&MyObject::data_), &MyObject::data_>("data", this);
    // We have the following macro helper
    MC_RTC_LOG_HELPER("something", something_);
    // Also works with methods
    MC_RTC_LOG_HELPER("computeVector", computeVector);
    // But for overloaded methods we must use another macro to work-around ambiguity issues
    MC_RTC_LOG_GETTER("referenceBody", referenceBody);
  }
};
```

The `MC_RTC_LOG_HELPER` and `MC_RTC_LOG_GETTER` make the following two assumptions:
- `this` is the logging source
- `logger` is the logger instance you are adding the data to

#### Python interface

The Python interface is very similar:

```python
# Add a log entry
self.logger().addLogEntry("my_entry", lambda: self.data)
# Remove data
self.logger().removeLogEntry("my_entry")
# Using partial to invoke a controller method
self.logger().addLogEntry("my_entry", partial(self.get_data))
```

#### Performance concern

By default, lambda expressions return by value. When possible, you might want to return a const reference to the object you are saving:

```cpp
logger().addLogEntry("entry", [this]() -> const sva::PTransformd & { return trans_; }
```

### Getting the log

Your controller will show the following message telling you what the latest log
is, the message should look like the following:

```console
Will log controller outputs to "/tmp/mc-control-MyControllerName-DATE.bin"
```

On systems that support symbolic links, it will also be available as `/tmp/mc-control-MyControllerName-latest.bin`

### Working with the log

See the following pages for tools that work with the log your controller generates:

- {% include link_tutorial.html category="tools" tutorial="mc_log_utils" %}
- {% include link_tutorial.html category="tools" tutorial="mc_log_ui" %}
- {% include link_tutorial.html category="tools" tutorial="mc_log_visualization" %}
