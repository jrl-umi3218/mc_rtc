---
layout: tutorials
---

`mc_convex_visualization` is a tool that allows you to visualize the convexes of a robot in the RViZ application. It is part of the {% link mc_rtc_ros %} package.

### Launching the visualization

The tool is launched as follows:

```bash
$ roslaunch mc_convex_visualization display.launch robot:=robot/jvrc1
```

After launching it should look like this:

<img src="img/mc_convex_visualization.png" alt="mc_convex_visualization in action" class="img-fluid" />

By enabling/disabling the checkboxes in the highlighted area you can show/hide selected convexes.
