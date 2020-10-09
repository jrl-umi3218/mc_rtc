---
layout: tutorials
toc: true
---

In this document we will assume you have a log at `~/my_log.bin`.

### `mc_bin_utils`: a general manipulation tool

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

#### `mc_bin_utils extract`

`mc_bin_utils extract` has two operation mode.

The first one extract all parts of a log where a given key is present:

```bash
$ mc_bin_utils extract ~/my_log.bin my_log_out --key com_task_target
```

This will generate one log for each section where the key `com_task_target` was present. For example, if the key was present at the 5 seconds mark until the 15 seconds mark and again from the 30 seconds mark to the 60 seconds mark you will get `my_log_out_1.bin` and `my_log_out_2.bin`.

The second form lets directly extract a time-range based on the `t` entry:

```bash
$ mc_bin_utils extract ~/my_log.bin my_log_out --from 50 --to 100
```

This example will generate `my_log_out.bin` with time `t` ranging from 50 seconds to 100 seconds. You can omit either `from` or `to` in which case the output log will start at 0 second and end at the end of the log respectively.

#### `mc_bin_utils convert`

This tool will let you convert a `.bin` log to one of three formats (two without ROS support)

- `.flat` is the actual format expected by `mc_log_ui` if you find yourself opening the same log frequently you may convert it once to save time;
- `.csv` is a well-known data format that is understood by a lot of tools (notably MATLAB and Excel) and can be convenient to share data with external collaborators;
- `.bag` is the format expected by the `rosbag` tool;

### Opening the log in Python

You can also open the log in Python:

```python
import mc_log_ui
log = mc_log_ui.read_log('/tmp/mylog.bin')
```

Then `log` is a Python dictionary where the keys are the entries you see in the tree of mc\_log\_ui (e.g. if you logged an `Eigen::Vector3d` called `v3d` then you get `v3d_x`, `v3d_y` and `v3d_z`) and the values are numpy arrays so you can do anything with it.

### The `mc_rtc::FlatLog` class

The `mc_rtc::FlatLog` class is part of the `mc_rtc_utils` library. It lets you open a log and iterate through the data using their original type. For example, if you have logged a force sensor's reading as an `sva::ForceVecd` object you can retrieve it as-is through the `mc_rtc::FlatLog`. See the class documentation for more details on the usage and API. This is only available in C++.
