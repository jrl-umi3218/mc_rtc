The constraint presented in this page is `mc_solver::BoundedSpeedConstr`. As its name and the page's title indicates, it can be used in a scenario where a body absolutely needs to move at a given speed in a given direction. For more "relaxed" approach, you can take a look at the following articles:

- [Affect different weight to different axis in the task space](dim-weight.html)
- [Change the DoF constraints on a contact](contact-dof.html)

## How to create the object and add speed constraints

The first step is to create the constraint object:

```cpp
// Assume bSpeedCstr type is:
// std::shared_ptr<mc_solver::BoundedSpeedConstr>
bSpeedCstr = std::make_shared<mc_solver::BoundedSpeedConstr>(robots(),
                                                             robots().robotIndex(),
                                                             timeStep);
```

Out of this parameter, the only one that you might change is `robotIndex` that will dictate upon which robot the constraint will act.


Then, we add it to the solver:

```cpp
solver().addConstraintSet(*bSpeedCstr);
```

At this point, just like with collision constraint, the constraint has no actual effect, we only setup the mechanism to activate it.

Finally, we can add a speed constraint to the solver:

```cpp
bSpeedCstr->addBoundedSpeed(solver(),
                            bodyName,
                            bodyPoint,
                            dof,
                            speed);
```

In the code above:
- `bodyName` is the name of the body upon which the constrain will be applied
- `bodyPoint` is a point (`Eigen::Vector3d`) in the body's frame that should be used as rotation reference. If `bodyPoint` is zero, then the body's origin is used
- `dof` is a DoF selection matrix, more on that below
- `speed` is a 6d speed constraint

Note that `speed` is expressed in the body's frame.

#### About the DoF matrix

The requirements for the DoF matrix are pretty close to those of the [DoFContact](contact-dof.html). These requirements are:
- it should be a square 6x6 matrix;
- only the diagonal elements should be set;
- a 0 represents a DoF that is not affected by the constraint, while a 1 represents a DoF affected by the constraint;
- the order is (x, y, z) rotations first and then (x, y, z) translations.

Another possibility is to construct a speed constraint with lower and upper bounds, this is done like so:

```cpp
bSpeedCstr->addBoundedSpeed(solver(),
                            bodyName,
                            bodyPoint,
                            dof,
                            lowerSpeed,
                            upperSpeed);
```

The arguments are the same as above except that the speed is constrained to remain between `lowerSpeed` and `upperSpeed` instead of being fixed at a given value.

## Examples

In this first example, we will only constraint the hand to move at a constant speed along its normal axis:

```cpp
Eigen::MatrixXd dof = Eigen::MatrixXd::Identity(6,6);
Eigen::VectorXd spd = Eigen::VectorXd::Zero(6);
spd(5) = 0.1;  // Move r_wrist in the z direction at constant speed 0.1 m/s
bSpeedConstr->addBoundedSpeed(solver(), "r_wrist",
                              Eigen::Vector3d::Zero(),
                              dof, spd);
```

You should observe the robot raising its hand up-to a point where it cannot do so anymore and the solver will then fail. This is partly why this constraint should be used sparsely.

A more sensible example is to use the constraint to limit a body speed. For now, we will remove the constraint add an `mc_tasks::EndEffectorTask` named `efTask` to the solver and give it a very high stiffness:

```cpp
efTask = std::make_shared<mc_tasks::EndEffectorTask>("r_wrist", robots(), 0);
efTask->set_ef_pose(sva::PTransformd(sva::RotY<double>(-M_PI/2),
                                     efTask->get_ef_pose().translation() + Eigen::Vector3d(0.3, -0.1, 0.2)));
efTask->positionTask->stiffness(100);
efTask->orientationTask->stiffness(100);
solver().addTask(efTask);
```

If you run this, you will see the robot's hand moving very suddenly. Of course, this movement is perfectly feasible by the actual robot (since other constraints are in place to ensure that) but we will limit the movement speed throught the bounded speed constraint.

```cpp
Eigen::MatrixXd dof = Eigen::MatrixXd::Identity(6,6);
Eigen::VectorXd spd = Eigen::VectorXd::Zero(6);
// Apply a different scaling for linear and angular velocities
for(size_t i = 0; i < 3; ++i) { spd(i) = M_PI*0.1; }
for(size_t i = 3; i < 6; ++i) { spd(i) = 0.1; }
bSpeedConstr->addBoundedSpeed(solver(), "r_wrist",
                              Eigen::Vector3d::Zero(),
                              dof, -spd, spd);
```

If you run this code now, you will see that the movement is much smoother.

**Note:** The above example is fairly contrived, this is not a good way to build a high stiffness/slow task.
