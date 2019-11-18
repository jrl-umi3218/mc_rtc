---
layout: tutorials
---

## What is a DoF contact?

A DoF contact, short for Degree of Freedom contact, is a facility that allows us to free some degress of freedom on a contact. To be more precise, whenever we have a contact, we can free movement along one or more axis (in translation and/or rotation).

This facility will mainly allow us to block motion in some directions while allowing it in other:
- Whenever inserting or removing a contact (flat or cylindrical) we usually want to only move along the normal axis
- When we have a flat contact, we may want to adjust orientation without changing the position to better match the real surface's orientation.

## How to create one?

DoF contacts will only modify the ContactConstraint. Let's see the interesting prototypes in `Tasks::QPContactConstr.h`:
```cpp
bool addDofContact(const ContactId& contactId, const Eigen::MatrixXd& dof);
bool removeDofContact(const ContactId& contactId);
void updateDofContacts();
void resetDofContacts();
```
Thus the typical flow of DoF constraint will go like this:
- Find the correct Contact to be freed/constrained
- Select the correct DoF
- Add it to the constraint
- Update the constraint
- When done, remove the DoF and update again

To find a ContactID, the best way is to create an `mc_rbdyn::Contact` and then call the `contactId` method like so:
```cpp
mc_rbdyn::Contact contact;
tasks::qp::contactId cId = contact.contactId(robots());
```

Now, how to select DoF:
- The DoF matrix should be a square 6x6 matrix
- Only the diagonal elements should be set in normal cases. Non-diagonal elements could be used to generate for example an "helicoidal" constraint.
- Every 0 element represents a free DoF, while every 1 represents a blocked DoF.
- The order is first the (x, y, z) rotations, then the (x, y, z) translations.

For example:
```
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0
```
Is a "virtual" contact: the motion is not restricted at all.

```
1 0 0 0 0 0
0 1 0 0 0 0
0 0 1 0 0 0
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
```

Is entirely blocked (regular contact)

```
1 0 0 0 0 0
0 1 0 0 0 0
0 0 1 0 0 0
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 0
```

Is blocked in every direction except the normal translation (useful for insertion).

```
1 0 0 0 0 0
0 1 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 1
```

Is a typical sliding planar contact: we can rotate around the normal axis, and translate along both tangential axis.

> The rotations center is at the center of the contacting surface. This can be a problem when trying to rotate a gripper around a cylinder, because the axis of rotation will not be aligned with the cylinder's axis.

## How about an example?

Let's start with our default example of HRP2 standing on the ground. In our constructor you should have:
```cpp
    solver().setContacts({
            {robots(), 0, 1, "LeftFoot", "AllGround"},
            {robots(), 0, 1, "RightFoot", "AllGround"}
            });
```

How about we try to move the right foot exactly vertically:

```cpp
//In reset
Eigen::Matrix6d dof = Eigen::Matrix6d::Identity();
dof(5,5) = 0;
 
tasks::qp::ContactId ContactId = mc_rbdyn::Contact(robots(), 0, 1, "RightFoot", "AllGround").contactId(robots());
contactConstraint().contactConstr->addDofContact(ContactId, dof);
contactConstraint().contactConstr->updateDofContacts();
```

Now let's add an EndEffectorTask that will try to move the foot *not* vertically and check that the constraint does its job.

```cpp
//In header
std::shared_ptr<mc_tasks::EndEffectorTask> efTask;

//In .cpp
// In constructor
efTask = std::make_shared<mc_tasks::EndEffectorTask>("R_FOOT", robots(), 0, 5.0, 100);
efTask->addToSolver(solver());

//In reset
efTask->resetTask(robots(), 0);
efTask->add_ef_pose(sva::PTransformd(Eigen::Vector3d(0.2, 0., 0.2)));
```
If you launch this you will see JVRC1 moving its foot perfectly vertically.
> Note: In this setup, you need to be in kinematics mode, else the robot will fall, unless you add a first step where you shift the robot's weight onto the left foot.

Let's slightly change this to show how this can be used to simulate a sliding planar contact. Let's just change the `dof` matrix, and the target of the `EndEffectorTask`.

```cpp
Eigen::Matrix6d dof = Eigen::Matrix6d::Identity();
dof(2,2) = 0;
dof(3,3) = 0;
dof(4,4) = 0;

efTask->add_ef_pose(sva::PTransformd(sva::RotX(1.)*sva::RotY(1.)*sva::RotZ(1.),Eigen::Vector3d(0.2, -0.2, 0.2)));
```

This time you will see that the contact stays horizontal, at the same height but moves in the plane and rotates around the vertical axis.

Again, this may only work in kinematics mode.
