---
layout: tutorials
---

`mc_surfaces_visualization` is a tool that allows you to visualize the surfaces of a robot in the RViZ application. It is part of the {% link mc_rtc_ros %} package.

### Launching the visualization

The tool is launched as follows:

```bash
$ roslaunch mc_surfaces_visualization display.launch robot:=JVRC1
```

The `robot` argument should be the same as what you would pass to the `mc_rbdyn::RobotLoader::get_robot_module` function, this can also be a robot alias. For example:

```bash
# Specify a vector of arguments
$ roslaunch mc_surfaces_visualization display.launch robot:="[env, `rospack find mc_env_description`, ground]"
# Or an alias
$ roslaunch mc_surfaces_visualization display.launch robot:=env/ground
```

After launching it should look like this:

<img src="img/mc_surfaces_visualization.png" alt="mc_surfaces_visualization in action" class="img-fluid" />

In the 3D display:

- Planar surfaces are drawn in green with a blue arrow showing the normal direction of the surface (as seen on the feet in the screenshot);
- Cylindrical surfaces are green cylinders (not seen in the screenshot);
- Gripper surfaces are represented with blue arrows representing the normal direction of the gripper's points' frame;

By enabling/disabling the checkboxes in the highlighted area you can show/hide selected surfaces.
