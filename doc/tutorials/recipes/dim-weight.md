---
layout: tutorials
toc: true
---

It is possible to affect different weights to different axis in the task-space (e.g. ignore the vertical position of the CoM).

Most tasks can take in an additional parameter `dimWeight` of type `Eigen::VectorXd` that allows you to weigh differently different directions:
- The `dimWeight` does not replace the total weight. It is multiplicative with it. For example, a `comTask` with weight `1000` and a `dimWeight` of `[1., 1., 1e-3]` will behave like a task of total weight `[1000., 1000., 1.]` 
- The `dimWeight` is a dynamic size vector. It is up to *you* to make sure it has the right size. Note that you can obtain the (lower-level) task size by calling `dim()` on it.
- You can pass this parameter at construction and/or change it later.

#### Example
To ignore the vertical error on a `comTask`:
```cpp
Eigen::VectorXd dimWeight(3);
dimWeight.setOnes();
dimWeight[2] = 0.;

comTask = std::make_shared<mc_tasks::CoMTask>(robots, robots().robotIndex(), 5.0, 100.);
comTask->dimWeight(dimWeight);
```

`EndEffectorTask` is composed of two tasks in order to control the orientation and the position of the end-effector. As such, its `dimWeight` method excepts a 6d vector, the first three dimensions control the orientation dimension weight while the last three control the position dimension weight. To ignore the `x` and `y` translations on an `EndEffectorTask`:
```cpp
Eigen::VectorXd dimWeight(6);
dimWeight << 1., 1., 1., 0., 0., 1.;

efTask = std::make_shared<mc_tasks::EndEffectorTask>("RARM_LINK6", robots(), 5.0, 100);
efTask->dimWeight(dimWeight);
```

Alternatively, you could address the position part of the task directly:
```cpp
Eigen::VectorXd dimWeight(3);
dimWeight.setZero();
dimWeight(2) = 1.;

efTask = std::make_shared<mc_tasks::EndEffectorTask>("RARM_LINK6", robots(), 5.0, 100);
efTask->positionTask->dimWeight(dimWeight);
```

> Note: this is *not* a constraint, it simply means that the task has no effect in those directions. If you want to constrain motion in a certain direction, you should use either a [DoF Contact](contact-dof.html) or a [Bounded Speed Constraint](speed-constraint.html).
