`mc_surfaces_visualization` is a tool that allows you to visualize the surfaces of a robot in `mc_rtc GUI` or the RViZ application. This is a sub-tool of [mc_robot_visualization]({{site.baseurl}}/tutorials/tools/mc_robot_visualization.html) provided for convenience.

### Launching the visualization

The tool is launched as follows:

```bash
$ mc_surfaces_visualization[_ros] JVRC1
```

Note that `mc_surfaces_visualization` only publishes in `mc_rtc gui`, while `mc_surfaces_visualization_ros` publishes in both `mc_rtc gui` and `ros` (RViZ).
If you installed from debian packages, the ros-free version is provided by `mc-rtc-utils` while the ros version is provided by `ros-<distro>-mc-rtc-utils`.


The arguments to the program should be the same as you would use as a `MainRobot` entry. It can also handle aliases. For example:

```bash
# Specify a vector of arguments
$ mc_surfaces_visualization_ros env `rospack find mc_env_description` ground
# Or an alias
$ mc_surfaces_visualization_ros env/ground
```

After launching an mc_rtc GUI, it should look like this:

<img src="{{site.baseurl_root}}/assets/tutorials/tools/img/mc_surfaces_visualization.png" alt="mc_surfaces_visualization in action" class="img-fluid" />

In the 3D display:

- Planar surfaces are drawn in green with a blue arrow showing the normal direction of the surface (as seen on the feet in the screenshot);
- Cylindrical surfaces are green cylinders (not seen in the screenshot);
- Gripper surfaces are represented with blue arrows representing the normal direction of the gripper's points' frame;

You can easily select which surfaces to display through the interface.
