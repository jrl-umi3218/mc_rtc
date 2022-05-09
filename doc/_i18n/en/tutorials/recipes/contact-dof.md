## What is a DoF contact?

A DoF contact, short for Degree of Freedom contact, is a facility that allows us to free some degress of freedom on a contact. To be more precise, whenever we have a contact, we can free movement along one or more axis (in translation and/or rotation).

This facility will mainly allow us to block motion in some directions while allowing it in other:
- Whenever inserting or removing a contact (flat or cylindrical) we usually want to only move along the normal axis
- When we have a flat contact, we may want to adjust orientation without changing the position to better match the real surface's orientation.

## How to create one?

To create one we need to have:
- a contact we wish to modify
- a correct DoF selection

To select DoF:
- The DoF vector is a 6x1 vector;
- Every 0 element represents a free DoF, while every 1 represents a blocked DoF.
- The order is first the (x, y, z) rotations, then the (x, y, z) translations.

For example:
```
0 0 0 0 0 0
```
Is a "virtual" contact: the motion is not restricted at all.

```
1 1 1 1 1 1
```

Is entirely blocked (regular contact)

```
1 1 1 1 1 0
```

Is blocked in every direction except the normal translation (useful for insertion).

```
1 1 0 0 0 1
```

Is a typical sliding planar contact: we can rotate around the normal axis, and translate along both tangential axis.

> The rotations center is at the center of the contacting surface. This can be a problem when trying to rotate a gripper around a cylinder, because the axis of rotation will not be aligned with the cylinder's axis.

## How about an example?

Let's start with our default example of JVRC1 standing on the ground. In our constructor you should have:
```cpp
addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
addContact({robot().name(), "ground", "RightFoot", "AllGround"});
```

How about we try to move the right foot exactly vertically:

```cpp
//In reset
Eigen::Vector6d dof = Eigen::Vector6d::Ones();
dof(5) = 0;
double friction = mc_rbdyn::Contact::defaultFriction;
addContact({robot().name(), "ground", "RightFoot", "AllGround", friction, dof});
```

Now let's add an EndEffectorTask that will try to move the foot *not* vertically and check that the constraint does its job.

```cpp
//In header
std::shared_ptr<mc_tasks::EndEffectorTask> efTask;

//In .cpp
// In constructor
efTask = std::make_shared<mc_tasks::EndEffectorTask>(robot().frame("RightFoot"), 5.0, 100);
solver().addTask(efTask);

//In reset
efTask->reset();
efTask->add_ef_pose(sva::PTransformd(Eigen::Vector3d(0.2, 0., 0.2)));
```
If you launch this you will see JVRC1 moving its foot perfectly vertically.
> Note: In this setup, you need to be in kinematics mode, else the robot will fall, unless you add a first step where you shift the robot's weight onto the left foot.

Let's slightly change this to show how this can be used to simulate a sliding planar contact. Let's just change the `dof` vector, and the target of the `EndEffectorTask`.

```cpp
Eigen::Vector6d dof = Eigen::Vector6d::Ones();
dof(2) = 0;
dof(3) = 0;
dof(4) = 0;

efTask->add_ef_pose(sva::PTransformd(sva::RotX(1.)*sva::RotY(1.)*sva::RotZ(1.),Eigen::Vector3d(0.2, -0.2, 0.2)));
```

This time you will see that the contact stays horizontal, at the same height but moves in the plane and rotates around the vertical axis.

Again, this may only work in kinematics mode.
