---
layout: tutorials
---

This guide will help you to create environment that are compatible with mc_rtc and other JRL/LIRMM softwares.

The screen captures of this guide are taken from Blender 2.78 but it should apply to any recent enough Blender version.

# Before starting

## Required softwares

1. Blender
2. [robot_script](https://gite.lirmm.fr/multi-contact/robot_script)

Concerning Blender, on Ubuntu systems, it is advised to use [Thomas Schiex's PPA](https://launchpad.net/~thomas-schiex/+archive/ubuntu/blender) in order to get a fully featured Blender installation. In particular, collada (`.dae` files) import is not supported in the repository version of Blender.i

`robot_script` is a collection of Blender and shell scripts we will be using to go from a Blender file to a fully functional environment for mc_rtc.

## Install our Blender addon

1. `mkdir -p $HOME/.config/blender/$(blender -v|awk '{print $2}')/scripts/addons`
2. Clone `robot_script` and navigate to the `robot_script` folder
3. `cp -r blender_addons/io_qconvex/ $HOME/.config/blender/$(blender -v|awk '{print $2}')/scripts/addons`
4. Start Blender
5. Activate the addon: "Import-Export: qconvex cloud format" as shown in the image below

<img src="img/blender_addon_activation.gif" alt="blender_addon_activation" class="img-fluid" />

Step 1 creates an empty folder to put our addon in while Step 3 simply copy that addon to that folder.

# Obtain your environment model

We will not go into the detail of modeling a 3D environment here. For the remainder of this documentation we will be working on the model provided [here](data/blender_model_start.blend). When you open this in Blender you should see a large flat surface (named "ground") with a vertical cylinder in the middle (named "pole").

<img src="img/blender_model_start.png" alt="blender_model_start" class="img-fluid" />

## A note on scale

Make sure your model meshes have a unary scale. You can do so by applying the scale parameter of every objects in your scene:

1. Select all scene objects (Shortcut: A)
2. Apply (Shortcut: Ctrl + A)
3. Select scale

# Create a planar surface

We will create two surfaces in this section, the first will showcase the basics to create a surface, the second will allow us to show an important issue about orientation.

## Creation of the Ground surface

For this first surface, we will create a surface corresponding to the ground.

1. Select the "ground" object (Right click on the ground mesh)
2. Go into Edit mode (Shortcut: Tab)
3. Select the upper face (Choose the face selection tool then right click on the face)
4. Duplicate the face (Shortcut: Shift + D) then hit escape to keep the face at its original position
5. Separate the face from the "ground" object (Shortcut: P, then choose "Selection")
6. This should have created a new object named "ground.001", rename this object Ground
7. Change the origin of the "Ground" object to its geometry
8. Select the "Ground" object then select the "ground" object (First click "Ground" in the scene hierarchy, then click "ground" while holding Shift)
9. Set "ground" as a parent to "Ground" (Shortcut: Ctrl + P, then choose "Keep transform")
10. Add a custom property named "surface" without any value to the "Ground" object

Congratulations you created you very first surface!

This [movie](https://gite.lirmm.fr/multi-contact/mc_rtc/wikis/videos/blender_planar_surface.mp4) will show this process.

#### A note on naming conventions

The following conventions are followed and recommended for most environments in mc_rtc:

1. Bodies' names should be in lower-case (e.g. "ground" or "stair_step_1")
2. Surfaces' names should be in CamelCase (e.g. "Ground" or "StairStep1")
3. Collision hulls should be in lower-case with the "\_hull" prefix (e.g. "ground_hull" or "stair_step_1_hull")

## Creation of the GroundSide surface

For this second surface, we will create a surface corresponding to one of the side of the ground. While it does not have any practical purpose, it will serve to illustrate the importance of orientation and how to change it. First, reproduce all the steps above but with a side of the ground mesh to create the surface "GroundSide".


If you were to generate the environment now, you would notice that surface is not oriented as you expect. This is because the local frame of the object is not correct. We need to make the Z-axis (blue in Blender) match the normal of the surface.

1. Select the "GroundSide" object
2. Make sure to enable the local frame display rather than global
3. Rotate the object 90 degrees around the Y-axis (Shortcut: R, then Y to lock the Y axis, then hit 90 on the numeric pad of your keyboard, then hit Enter)
4. Go into Edit mode (Shortcut: Tab)
5. Select all points in the mesh (Shortcut: A)
6. Rotate the points -90 degrees around the Y-axis

Congratulations, you fixed the orientation issue!

This [movie](https://gite.lirmm.fr/multi-contact/mc_rtc/wikis/videos/blender_rotate_surface.mp4) will show this process.

## Planar surface creation caveats

Some caveats apply when designing a planar surface.

When creating a contact between the surface you designed and another surface, mc_rtc will generate contact forces at the points of your surface. This means that the more points you put into a surface, the more variable will be added to the problem. Thus, it is best to keep your surface design simple (i.e. a single quadrilater), especially when you are designing a robot surface.

The points need to be co-planar otherwise the environment generation will fail.

# Create a cylindrical surface

In this section we will create a single cylindrical surface. The same orientation issue applies than with planar surface. Here the convention is that the Z-axis is the cylinder's axis and the X-axis is the cylinder's polar axis.

1. Select the "pole" object
2. Snap the cursor to the object origin (Shortcut: Shift + S, then Cursor to selected)
3. Add a plain axes object and rename it "Pole"
4. Make "pole" the parent of "Pole"
5. Add a custom property named "surface" to the object with the value "cylindrical"
6. Add a custom property named "radius", the value of this property will be the radius of the cylinder
7. Add a custom property named "width", the value of this property will be the length of the cylinder

Congratulations, you created your first cylindrical surface!

This [movie](https://gite.lirmm.fr/multi-contact/mc_rtc/wikis/videos/blender_create_cylindrical.mp4) will show this process.

Note that by default, Blender constrains custom properties value in the [0, 1] range. Be careful when setting your cylinder "width" property as this might be a problem.

# Add a collision hull

To add a collision hull simply add a new mesh and add an empty property named "hull".

Note that the generation script will create convex hulls for each body in the model, you don't need to create specific hulls for those.

Before going into the next step, you can get the result of the tutorial [here](data/blender_model_final.blend).

# Generate the environment

Go into the robot_script folder and launch the do_all.sh script as such:

```bash
./do_all.sh my_file.blend /path/to/ros/package env_name package_name
```

The package_name is optional if you are adding the environment to the `mc_env_description` package.

# Testing the environment

See the tutorial about [visualizing surfaces]({{site.baseurl}}/tutorials/tools/mc_surfaces_visualization.html).

## Common problems

#### The surface location is wrong

Make sure the origin of the surface object in the Blender project corresponds to what you expect.

#### The surface orientation is wrong

Make sure the surface normal is facing the right direction in the Blender project. Make sure that you are viewing the local frame of the object.

# Add a surface to a robot

To add a surface to a robot, you would follow the same steps as with an environment but the environment model is the robot's body where you want to add a surface. Then generate the "environment" into a temporary directory and use the generated rsdf file in your original robot's description folder.

