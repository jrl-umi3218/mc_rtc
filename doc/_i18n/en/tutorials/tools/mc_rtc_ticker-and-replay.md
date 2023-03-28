We have introduced the use of mc_rtc_ticker in the {% include link_tutorial.html category="introduction" tutorial="running-a-controller" %} tutorial. In this section we will highlight the extra functionalities available in mc_rtc_ticker, the replay option it offers and the use of the "Replay" plugin.

#### mc_rtc_ticker options

mc_rtc_ticker accepts the following options:

{% highlight bash %}
$ mc_rtc_ticker --help
mc_rtc_ticker options:
  --help                           Show this help message
  -f [ --mc-config ] arg           Configuration given to mc_rtc
  -S [ --step-by-step ]            Start the ticker in step-by-step mode
  --run-for arg                    Run for the specified time (seconds)
  -s [ --no-sync ]                 Synchronize ticker time with real time
  -r [ --sync-ratio ] arg          Sim/real ratio for synchronization purpose
  -l [ --replay-log ] arg          Log to replay
  -m [ --datastore-mapping ] arg   Mapping of log keys to datastore
  -g [ --replay-gui-inputs-only ]  Only replay the GUI inputs
  -e [ --exit-after-replay ]       Exit after log replay
  --replay-outputs                 Enable outputs replay (override controller)
{% endhighlight %}

##### `-f [file]`/`--mc-config [file]`

This option allows to override the mc_rtc configuration file used to configure mc_rtc. The provided `file` is used to configure mc_rtc after the usual files.

##### `-S`/`--step-by-step`

This option starts the controller in step-by-step mode. In this mode, the controller does not run unless it is instructed to step for a number of iterations. This can be done via the GUI.

##### `--run-for [time]`

Run the ticker for the specified `time` duration (in seconds) and exit.

##### `-s`/`--no-sync`

Normally the ticker synchronizes the real-time clock and the simulation time such that if one second passed in real-time, one second passed in the simulation. With this option, the synchronization mechanism is disabled and the simulation time will run as fast as possible.

##### `-r [ratio]`/`--sync-ratio [ratio]`

Controls the target simulation-time/real-time ratio. For example, with a value of 0.5, when one second passes in the simulation, 2 seconds have passed in real-time but with a ratio of 2, when one second passes in the simulation, 0.5 second will have passed in real-time (provided the CPU is able to run that fast).

#### Replay functionality

The following options are related to the replay functionality. In replay mode, a log is loaded and can be used to provide some data to the controller.

##### `-l [log]`/`--replay-log[log]`

This option enables the replay-mode using the provided `log` argument as a log. By default, this will replay all sensors' inputs and GUI interaction with and run the plugins, observation pipelines and controllers with these inputs. The behavior of the replay mode can be fine-tuned by the following options.

##### `-m [yaml]`/`--datastore-mapping [yaml]`

This option allows to specify a log-to-datastore mapping. The replay will take data from the log and put them in the datastore. The type of data presented in the datastore depends on the type of data in the log.

Example configuration:

{% highlight yaml %}
# Keys are entry in the log, values are entry in the datastore
ff: Log::FloatingBaseControl
ff_real: Log::FloatingBaseReal
{% endhighlight %}

The intent of this functionality is to allow to replay inputs data from systems that are not necessarily connected to the controller during replay.

##### `-g`/`--replay-gui-inputs-only`

Instead of replaying both the sensors inputs and the GUI inputs, this replays the GUI inputs only.

##### `-e`/`--exit-after-replay`

Exit the ticker at the end of the replay. The default behavior is to enable the step-by-step mode at the end of the log.

##### `--replay-outputs`

This option removes the controller from the loop and replay the outputs of the previous controller.

#### Using the Replay plugin

In addition to the Replay mode of the ticker, a `Replay` plugin is also available. The plugin can be enabled and configured by adding the following entries to either the configuration of mc_rtc or the configuration of the controller:

{% highlight yaml %}
Plugins: [Replay]
Replay:
  log: /path/to/log.bin
  with-inputs: true
  with-gui-inputs: true
  with-outputs: false
  with-datastore-config: /path/to/datastore-to-replay.yaml
{% endhighlight %}

The most common configuration of this replay plugin is to replay GUI inputs from a previous log in a simulation environment.

#### Detecting the Replay is active

When a replay is being played in the controller, either through the ticker's replay or via the Replay plugin, the `Replay::Log` entry is available in the controller and contains a shared pointer to the `mc_rtc::log::FlatLog` object being replayed.
