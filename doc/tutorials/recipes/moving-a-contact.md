---
layout: tutorials
toc: true
---

Through the various tutorials, you should have an idea of what contacts are, how they are handled in the controller and how you could move a contact. Let's say that you want to move the left foot from its starting position to another a bit farther away.

The essential steps to do so would be:
1. Move the CoM above the right foot;
2. Remove the left foot contact;
3. Move the left foot to the target destination;
4. Put the contact back;

Let's implement this simply.

## Simple implementation

We will need the following elements in our controller:

```cpp
// In the header
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/SurfaceTransformTask.h>
// If we want to use elements in mc_tasks or mc_solver we first must include in header file

std::shared_ptr<mc_tasks::CoMTask> comTask;
std::shared_ptr<mc_tasks::SurfaceTransformTask> footTask;

// This is not a good practice
bool moved_com = false;
bool moved_left_foot = false;
```

```cpp
void MyController::reset(const ControllerResetData & data)
{
  MCController::reset(data);
  /* Create the task */
  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0);
  footTask = std::make_shared<mc_tasks::EndEffectorTask>("LeftFoot", robots(), 0);
  /* Move the CoM above the right foot */
  comTask->com(comTask->com() + Eigen::Vector3d(0, -1*footTask->get_ef_pose().translation().y(), 0));
  /* Add the CoM task to the solver */
  solver().addTask(comTask);
}

bool MyController::run()
{
  bool ret = MCController::run();
  if(!moved_com)
  {
    if(comTask->eval().norm() < 5e-2 && comTask->speed().norm() < 1e-4)
    {
      mc_rtc::log::success("Moved the CoM");
      moved_com = true;
      solver().setContacts({
        mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")
      });
      solver().addTask(footTask);
      footTask->target(sva::PTransformd(Eigen::Vector3d(0.4, 0, 0)));
    }
    return ret;
  }
  if(moved_com && !moved_left_foot)
  {
    if(footTask->eval().norm() < 5e-2 && footTask->speed().norm() < 1e-4)
    {
      mc_rtc::log::success("Moved the left foot");
      moved_left_foot = true;
      solver().setContacts({
          mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"),
          mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")
      });
      solver().removeTask(footTask);
      auto com = comTask->com();

      com.x() =
        (robot().surfacePose("LeftFoot").translation().x()
         +
         robot().surfacePose("RightFoot").translation().x())/2;

      com.y() = 0;
      comTask->com(com);
    }
    return ret;
  }
  return ret;
}
```

What you will notice is that the robot "slides" its feet along the floor. That's because this is the most direct way to go to the desired position. There is many ways this could be solved but we will now cover two useful `MetaTask` that take care of such scenarios:

- `AddRemoveContactTask` which we will use to remove and add the left foot contact
- `MoveContactTask` which we will use to move the foot between the two points

## Using `AddRemoveContactTask`

To use `AddRemoveContactTask` we need a [BoundedSpeedConstraint]({{site.baseurl}}/tutorials/recipes/speed-constraint.html) in the solver. We will also need the task itself. Finally, we need two supplementary steps to remove and add back the foot. So, we will modify our controller header and the `reset()` function accordingly:

```cpp
// In the header
#include <mc_solver/BoundedSpeedConstr.h>
#include <mc_tasks/AddRemoveContactTask.h>
std::shared_ptr<mc_solver::BoundedSpeedConstr> bSpeedConstr;
std::shared_ptr<mc_tasks::AddRemoveContactTask> aRContactTask;
bool removed_left_foot = false;
bool added_left_foot = false;
```

```cpp
// In reset
bSpeedConstr = std::make_shared<mc_solver::BoundedSpeedConstr>(robots(), 0, timeStep);
solver().addConstraintSet(*bSpeedConstr);
```

For the removal, we will use the `RemoveContactTask` specialization of `AddRemoveContactTask`, the prototype is:
```cpp
  RemoveContactTask(mc_rbdyn::Robots & robots,
                 std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                 mc_rbdyn::Contact & contact,
                 double speed, double stiffness, double weight,
                 Eigen::Vector3d * userT_0_s = nullptr);
```

To add the contact, we will use the `AddContactTask` specialization, whose prototype is:
```cpp
  AddContactTask(mc_rbdyn::Robots & robots,
                 std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                 mc_rbdyn::Contact & contact,
                 double speed, double stiffness, double weight,
                 Eigen::Vector3d * userT_0_s = nullptr);
```

For both cases, the arguments are pretty similar, the most importants are:
- `mc_rbdyn::Contact & contact` which should be the contact we want to add or remove
- `double speed` which is the insertion/removal speed

Note that, this task does not take care of actually maintaing the contact list in the solver nor does it take care of collision avoidance so we will need to take care of that ourselves.

```cpp
bool MyController::run()
{
  bool ret = MCController::run();
  if(!moved_com)
  {
    if(comTask->eval().norm() < 5e-2 && comTask->speed().norm() < 1e-4)
    {
      mc_rtc::log::success("Moved the CoM");
      moved_com = true;
      solver().setContacts({
        mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")
      });
      mc_rbdyn::Contact rmContact(robots(), "LeftFoot", "AllGround");
      aRContactTask.reset(new mc_tasks::RemoveContactTask(robots(),
                                                          bSpeedConstr,
                                                          rmContact,
                                                          0.05,
                                                          2.0, 1000));
      solver().addTask(aRContactTask);
    }
    return ret;
  }
  if(moved_com && !removed_left_foot)
  {
    /* Monitor the foot altitude */
    double lf_z = robot().surfacePose("LeftFoot").translation().z();
    double rf_z = robot().surfacePose("RightFoot").translation().z();
    if(lf_z > rf_z + 0.05)
    {
      mc_rtc::log::success("Left foot contact removed");
      removed_left_foot = true;
      solver().removeTask(aRContactTask);
      solver().addTask(footTask);
      footTask->add_ef_pose(sva::PTransformd(Eigen::Vector3d(0.4, 0, 0)));
    }
    return ret;
  }
  if(removed_left_foot && !moved_left_foot)
  {
    if(footTask->eval().norm() < 5e-2 && footTask->speed().norm() < 1e-4)
    {
      mc_rtc::log::success("Moved the left foot");
      moved_left_foot = true;
      solver().removeTask(footTask);
      mc_rbdyn::Contact addContact(robots(), "LeftFoot", "AllGround");
      aRContactTask.reset(new mc_tasks::AddContactTask(robots(),
                                                       bSpeedConstr,
                                                       addContact,
                                                       0.05,
                                                       2.0, 1000));
      solver().addTask(aRContactTask);
      return ret;
    }
  }
  if(moved_left_foot && !added_left_foot)
  {
    /* Monitor the foot altitude */
    double lf_z = robot().surfacePose("LeftFoot").translation().z();
    double rf_z = robot().surfacePose("RightFoot").translation().z();
    if(fabs(lf_z - rf_z) < 1e-4)
    {
      mc_rtc::log::success("Added left foot");
      added_left_foot = true;
      solver().removeTask(aRContactTask);
      solver().setContacts({
          mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"),
          mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")
      });
      auto com = comTask->com();

      com.x() =
        (robot().surfacePose("LeftFoot").translation().x()
         +
         robot().surfacePose("RightFoot").translation().x())/2;

      com.y() = 0;
      comTask->com(com);
    }
  }
  return ret;
}
```
