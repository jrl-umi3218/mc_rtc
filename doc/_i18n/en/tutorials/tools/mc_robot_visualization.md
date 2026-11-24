`mc_robot_visualization` is a tool that allows you to visualize the entire robot modules and its state in `mc_rtc gui` or in ROS (RViZ).

### Launching the visualization

The tool is launched as follows:

```bash
$ mc_robot_visualization[_ros] JVRC1
```

Note that `mc_robot_visualization` only publishes in `mc_rtc gui`, while `mc_robot_visualization_ros` publishes in both `mc_rtc gui` and `ros` (RViZ).
If you installed from debian packages, the ros-free version is provided by `mc-rtc-utils` while the ros version is provided by `ros-<distro>-mc-rtc-utils`.

The arguments to the program should be the same as you would use as a `MainRobot` entry. It can also handle aliases. For example:

```bash
# Specify a vector of arguments
$ mc_robot_visualization_ros env `rospack find mc_env_description` JVRC1
# Or an alias
$ mc_robot_visualization_ros env/JVRC1
```

In the 3D display:

- The robot model is shown with its current pose and joint configuration;
- Links and joints are visualized according to the robot's URDF;
- Additional overlays may show sensor data, collision shapes, or other information depending on your configuration. You can visualize these overlays through the GUI, or with the shorthand version [mc_surfaces_visualization]({{site.baseurl}}/tutorials/tools/mc_surfaces_visualization.html) and [mc_convex_visualization]({{site.baseurl}}/tutorials/tools/mc_convex_visualization.html).

You can easily select which parts of the robot to display through the interface.
