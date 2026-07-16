This guide will help you to create environments that are compatible with mc_rtc and other JRL/LIRMM softwares.

Surfaces are created and exported in Blender using the [RSDF Surface Tool](https://github.com/isri-aist/rsdf_surface_exporter) addon, which lets you create, visualize and export planar and cylindrical RSDF surfaces directly from a Blender mesh, with no need to hand-edit custom properties.

<img src="{{site.baseurl_root}}/assets/tutorials/advanced/img/rsdf_exporter_demo.gif" alt="rsdf_exporter_demo" class="img-fluid" />

# Before starting

## Required softwares

1. A recent version of Blender (4.2 or newer)
2. The [RSDF Surface Tool](https://github.com/isri-aist/rsdf_surface_exporter) Blender addon, used in this guide to create and export surfaces, as well as to export the bodies and meshes of your environment

## Install the RSDF Surface Tool addon

1. Download the latest release from the [releases page](https://github.com/isri-aist/rsdf_surface_exporter/releases/latest) (or clone the repository and run `blender --command extension build` inside it to produce the `.zip` yourself)
2. Open Blender → **Edit → Preferences → Add-ons → Install…**
3. Select the downloaded `.zip` file
4. Enable the addon

Once enabled, a new **RSDF** tab appears in the 3D View sidebar (press **N** to toggle the sidebar if it is hidden).

# Obtain your environment model

We will not go into the detail of modeling a 3D environment here. For the remainder of this documentation we will be working on a model with a large flat surface (named "ground") with a vertical cylinder in the middle (named "pole"), matching the example shown in the animation above.

## A note on scale

Make sure your model meshes have a unary scale. You can do so by applying the scale parameter of every objects in your scene:

1. Select all scene objects (Shortcut: A)
2. Apply (Shortcut: Ctrl + A)
3. Select scale

# Create a planar surface

For this first surface, we will create a surface corresponding to the ground.

1. Select the "ground" object
2. Go into Edit mode (Shortcut: Tab)
3. Select the face(s) that should make up the surface (Choose the face selection tool then right click on the face)
4. In the RSDF sidebar tab, click **Add Plane**

A new, green, semi-transparent object appears immediately, matching the shape of the face(s) you selected, and a corresponding entry is added to the surface list in the sidebar. There is no more need to duplicate/separate the face, set a custom origin, or parent anything by hand: the addon computes the surface's origin and orientation for you directly from the selected geometry.

Congratulations, you created your very first surface!

#### A note on naming conventions

The following conventions are followed and recommended for most environments in mc_rtc:

1. Bodies' names should be in lower-case (e.g. "ground" or "stair_step_1")
2. Surfaces' names should be in CamelCase (e.g. "Ground" or "StairStep1")
3. Collision hulls should be in lower-case with the "\_hull" prefix (e.g. "ground_hull" or "stair_step_1_hull")

Surfaces created by the addon are named "Surface_N" by default. Select the surface in the list and edit its **Name** field in the sidebar to rename it according to these conventions.

## Planar surface creation caveats

Some caveats apply when designing a planar surface.

When creating a contact between the surface you designed and another surface, mc_rtc will generate contact forces at the points of your surface. This means that the more points you put into a surface, the more variable will be added to the problem. Thus, it is best to keep your surface design simple (i.e. a single quadrilater), especially when you are designing a robot surface.

The points need to be co-planar otherwise the surface will not be usable.

# Create a cylindrical surface

In this section we will create a single cylindrical surface out of the "pole" object. The convention is that the local X-axis is the cylinder's polar axis; the addon takes care of orienting the surface correctly for you, you no longer need to reason about local frames or apply manual rotations.

1. Select the "pole" object
2. Go into Edit mode (Shortcut: Tab)
3. Select all the faces that make up the cylindrical band (Shortcut: A to select the whole mesh, if the object only contains the cylinder)
4. In the RSDF sidebar tab, click **Add Cylinder**

The addon fits a cylinder (center, axis, radius and width) to your selection and creates the corresponding green surface immediately, with its radius and width computed directly from your selected geometry.

Congratulations, you created your first cylindrical surface!

Unlike the previous, custom-property based workflow, the surface's radius and width are regular numeric fields internally, so there is no risk of them being silently clamped to the [0, 1] range.

# Export your environment as a ROS2 package

You no longer need a separate tool to export your environment's bodies. Once your bodies are ready in Blender, the RSDF Surface Tool addon can export the whole environment as a ready-to-use ROS2 package.

1. In the RSDF sidebar tab, click **Export ROS2 Description Package**
2. Set the package name, robot name, maintainer info, and whether to include the RSDF surfaces you created (**Include RSDF Surfaces**)
3. Choose a destination folder

This generates a full `ament_cmake` package with the following layout:

```
<package_name>/
  package.xml
  CMakeLists.txt
  urdf/<robot_name>.urdf
  meshes/<link_name>.stl
  rsdf/<robot_name>.rsdf        (if "Include RSDF Surfaces" is enabled)
  launch/display.launch.py
  launch/display.rviz
```

Every top-level mesh/empty object in the scene becomes a URDF link named after the object, and parent/child relationships become URDF joints. Mesh geometry is exported as STL in each object's own local space, so it lines up with the joint origins.

#### A note on joints

By default, every joint is **fixed**. To make a joint movable, add custom properties to the child object:

1. Select the child object in the 3D viewport
2. In the **Properties editor**, open the **Object Properties** tab (the small orange square icon, not the green triangle "Object Data" tab)
3. Scroll to the bottom of that tab and expand the **Custom Properties** section
4. Click **+ Add** (or **New**) once per property below, rename it to the exact name, and set its value. Every movable joint needs a `joint` property naming its type, plus the properties required for that type:

| `joint` value | Meaning | Required properties | Optional properties |
|---|---|---|---|
| `revolute` | Bounded rotation around `axis` | `axis`, `lower`, `upper` | `effort`, `velocity`, `name` |
| `continuous` | Unbounded rotation around `axis` (e.g. a wheel) | `axis` | `effort`, `velocity`, `name` |
| `prismatic` | Sliding motion along `axis` | `axis`, `lower`, `upper` | `effort`, `velocity`, `name` |

- `axis`: string `"x y z"`, e.g. `"0 0 1"` to rotate/slide around Z
- `lower`, `upper`: floats, joint limits in radians (revolute) or metres (prismatic)
- `effort`: float, default `1000`, max effort (N or N·m) reported in the URDF `<limit>` tag
- `velocity`: float, default `1.0`, max velocity (rad/s or m/s) reported in the URDF `<limit>` tag
- `name`: string, optional, overrides the joint's name in the URDF (defaults to `<link_name>_joint`)

If `joint` is omitted, the object gets a fixed joint. If `joint` is set but a required property for that type is missing, the export falls back to a fixed joint and reports a warning listing the missing property.

# Collision hulls

You don't need to create collision hulls by hand: mc_rtc automatically generates convex hulls from the mesh-type collision geometries referenced in the URDF, regardless of how the robot or environment is loaded (`env`/`object`, `json`, or C++ `RobotModule`). See [Automatic Convex Hull Generation]({{site.baseurl}}/tutorials/advanced/new-robot.html) in the "Integrate a new robot" tutorial for details on how this works and where the generated hulls are cached.

# Export and use your surfaces

Once you are satisfied with your surfaces, click **Export RSDF** in the sidebar and choose a filename and location; if you forget the `.rsdf` extension it will be appended for you automatically.

mc_rtc looks for surfaces in a robot or environment's `<path>/rsdf/<name>/` directory, loading every `*.rsdf` file it finds there and merging their surfaces. Place your exported file(s) in that directory.

#### A note on links

Every surface exported by the addon is currently attached to the body named `base_link` (this is a known current limitation of the addon, see its readme). If your environment or robot has several distinct bodies, open the exported `.rsdf` file and edit the `link="..."` attribute of each `<planar_surface>`/`<cylindrical_surface>` element so that it matches the actual body name (e.g. "ground" or "pole") the surface belongs to.

# Testing the environment

See the tutorial about [visualizing surfaces]({{site.baseurl}}/tutorials/tools/mc_surfaces_visualization.html). You can also use the addon's own **Load RSDF** button to re-import a `.rsdf` file into Blender at any time, to double-check that a surface's position, orientation and type were exported correctly.

## Common problems

#### The surface location is wrong

Make sure the origin of the object/face you selected in the Blender project corresponds to what you expect before clicking **Add Plane**/**Add Cylinder**.

#### The surface orientation is wrong

Make sure the mesh normals are consistent (Blender's face selection uses the face winding to determine orientation).

#### Add Plane/Add Cylinder reports "Need at least 3 vertices"

Make sure you are in Edit mode with at least one face selected on the active object before clicking the button.

# Add a surface to a robot

To add a surface to a robot, follow the same steps as with an environment, but select faces on the robot body's own mesh instead of an environment mesh, then export and place the resulting `.rsdf` file in the robot description's `rsdf/<robot_name>/` directory, making sure the `link` attribute of each surface matches the body it belongs to.
