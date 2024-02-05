`mc_surfaces_visualization` is a tool that allows you to visualize the surfaces of a robot in the RViZ application. It is part of the {% link mc_rtc_ros %} package.

### Launching the visualization

The tool is launched as follows:

```bash
$ mc_surfaces_visualization JVRC1
```

The arguments to the program should be the same as you would use as a `MainRobot` entry. It can also handle aliases. For example:

```bash
# Specify a vector of arguments
$ mc_surfaces_visualization env `rospack find mc_env_description` ground
# Or an alias
$ mc_surfaces_visualization env/ground
```

After launching an mc_rtc GUI, it should look like this:

<img src="{{site.baseurl_root}}/assets/tutorials/tools/img/mc_surfaces_visualization.png" alt="mc_surfaces_visualization in action" class="img-fluid" />

In the 3D display:

- Planar surfaces are drawn in green with a blue arrow showing the normal direction of the surface (as seen on the feet in the screenshot);
- Cylindrical surfaces are green cylinders (not seen in the screenshot);
- Gripper surfaces are represented with blue arrows representing the normal direction of the gripper's points' frame;

You can easily select which surfaces to display through the interface.
