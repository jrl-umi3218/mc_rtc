`mc_convex_visualization` is a tool that allows you to visualize the convexes of a robot in any mc_rtc GUI application.

### Launching the visualization

The tool is launched as follows:

```bash
$ mc_convex_visualization JVRC1
```

The arguments to the program should be the same as you would use as a `MainRobot` entry. It can also handle aliases. For example:

```bash
# Specify a vector of arguments
$ mc_convex_visualization env `rospack find mc_env_description` ground
# Or an alias
$ mc_convex_visualization env/ground
```

After launching an mc_rtc GUI, it should look like this:

<img src="{{site.baseurl_root}}/assets/tutorials/tools/img/mc_convex_visualization.png" alt="mc_convex_visualization in action" class="img-fluid" />

You can easily select which convexes to display through the interface.
