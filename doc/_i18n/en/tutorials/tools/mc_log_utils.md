In this document we will assume you have a log at `~/my_log.bin`.

## General manipulation tool `mc_bin_utils`

`mc_bin_utils` lets you perform a number of operations on a binary log:

```bash
$ mc_bin_utils --help
mc_bin_utils is a command-line tool to work with mc_rtc bin logs

Available commands:
    show      Display information about the log
    split     Split a log into N part
    extract   Extra part of a log
    convert   Convert binary logs to various formats

Use mc_bin_utils <command> --help for usage of each command
```

We will covert the `extract` and `convert` commands. `show` and `split` are self-explanatory.

### `mc_bin_utils extract`

`mc_bin_utils extract` has three operation modes.

#### Extract based on key presence `--key`

The first mode extracts all parts of a log where a given key is present:

```bash
$ mc_bin_utils extract ~/my_log.bin my_log_out --key com_task_target
```

This will generate one log for each section where the key `com_task_target` was present. For example, if the key was present at the 5 seconds mark until the 15 seconds mark and again from the 30 seconds mark to the 60 seconds mark you will get `my_log_out_1.bin` and `my_log_out_2.bin`.

#### Extract based on a time range `--from --to`

The second mode extracts a time-range based on the `t` entry:

```bash
$ mc_bin_utils extract ~/my_log.bin my_log_out --from 50 --to 100
```

This example will generate `my_log_out.bin` with time `t` ranging from 50 seconds to 100 seconds. You can omit either `from` or `to` in which case the output log will start at 0 second and end at the end of the log respectively.

#### Extract a selection of keys `--keys`

The third mode extracts a set of selected keys from the log:

```bash
$ mc_bin_utils extract ~/my_log.bin my_log_out --keys RightFootForceSensor com_target com_eval
```

This example will generate a `.bin` file which only contains data for the specified keys -- as well as the `t` key which is always added. It will create multiple files if there is a range in the provided log file that does not have any of the provided key.

The command also supports wildcards:

```bash
$ mc_bin_utils extract ~/my_log.bin my_log_out --keys com_*
```

This example will extract every key that starts with `com_`.

### `mc_bin_utils convert`

This tool will let you convert a `.bin` log to one of three formats (two without ROS support)

- `.flat` is the actual format expected by `mc_log_ui` if you find yourself opening the same log frequently you may convert it once to save time;
- `.csv` is a well-known data format that is understood by a lot of tools (notably MATLAB and Excel) and can be convenient to share data with external collaborators;
- `.bag` is the format expected by the `rosbag` tool;

Stand-alone tools are also always to perform the same conversions:

- `mc_bin_to_flat` converts a `.bin` file to a `.flat` file;
- `mc_bin_to_log` converts a `.bin` file to a `.csv` file;
- `mc_bin_to_rosbag` converts a `.bin` file to a `.bag` file;

## Performance at a glance `mc_bin_perf`

`mc_bin_perf` can be used to quickly grab statistics about mc_rtc performances. It will look at all the entries starting with `perf_` in the log and output their average value and its standard deviation as well as the minimum and maximum values.

For example:

```bash
$ mc_bin_perf /tmp/mc-control-CoM-latest.bin
----------------------------------------------------------------
|                     |  Average |    StdEv |      Min |   Max |
----------------------------------------------------------------
|       ControllerRun |    0.451 |   0.0639 |    0.378 |  1.14 |
|       FrameworkCost |     7.64 |     2.81 |     2.55 |  52.8 |
|           GlobalRun |    0.488 |   0.0703 |    0.401 |  1.18 |
|                 Gui |   0.0035 |  0.00941 | 0.000543 | 0.322 |
|                 Log |  0.00732 |  0.00476 |  0.00563 | 0.163 |
|        ObserversRun | 6.79e-05 | 0.000215 |  2.4e-05 | 0.014 |
|   Plugins_ROS_after |   0.0137 |   0.0124 |  0.00202 | 0.172 |
| SolverBuildAndSolve |    0.422 |   0.0607 |    0.354 |  1.11 |
|         SolverSolve |     0.32 |   0.0498 |    0.268 |     1 |
----------------------------------------------------------------
```

All times are presented in milliseconds -- except `FrameworkCost` as explained below.

Of the entries presented by default the following should be more interesting to the regular user of mc_rtc:

- `GlobalRun` is the total time spent in `mc_control::MCGlobalController::run()`, i.e. the time to run everything in mc_rtc: global plugins, observer pipelines, controller, logging and GUI. It is roughly equal to `Plugins_* + ObserversRun + ControllerRun + Gui + Log`;
- `ControllerRun` is the time spent in the controller run function. It is roughly equal to `SolverBuildAndSolve + (time spent in controller code)`;
- `SolverBuildAndSolve` and `SolverSolve` are the time spent in `Tasks` -- `SolverSolve` is included in `SolverBuildAndSolve` it is measuring the time spent solving the underlying QP problem;
- `FrameworkCost` is the ratio of time spent outside of `Tasks`, i.e. `(GlobalRun - SolverBuildAndSolve) / GlobalRun`;

## Opening the log in Python

You can also open the log in Python:

```python
import mc_log_ui
log = mc_log_ui.read_log('/tmp/mylog.bin')
```

Then `log` is a Python dictionary where the keys are the entries you see in the tree of mc\_log\_ui (e.g. if you logged an `Eigen::Vector3d` called `v3d` then you get `v3d_x`, `v3d_y` and `v3d_z`) and the values are numpy arrays so you can do anything with it.

## The `mc_rtc::log::FlatLog` class

The {% doxygen mc_rtc::log::FlatLog %} class is part of the `mc_rtc_utils` library. It lets you open a log and iterate through the data using their original type. For example, if you have logged a force sensor's reading as an `sva::ForceVecd` object you can retrieve it as-is through the `mc_rtc::log::FlatLog`. See the class documentation for more details on the usage and API. This is only available in C++.
