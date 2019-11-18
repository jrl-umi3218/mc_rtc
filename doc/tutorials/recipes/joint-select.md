---
layout: tutorials
---

It is possible to specify a subset of joints to perform a given task (e.g. one might want to perform stabilization using only the lower body).

In the `MetaTask` interface, this is implemented using a set of three functions:

```cpp
void selectActiveJoints(mc_solver::QPSolver & solver,
                        const std::vector<std::string> & activeJointsName);

void selectUnactiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & unactiveJointsName);

void resetJointsSelector(mc_solver::QPSolver & solver);
```

## Examples

A CoM task that only stimulates the legs:
```cpp
std::vector<std::string> activeJoints = {"RLEG_JOINT0", "RLEG_JOINT1", "RLEG_JOINT2",
                                         "RLEG_JOINT3", "RLEG_JOINT4", "RLEG_JOINT5",
                                         "LLEG_JOINT0", "LLEG_JOINT1", "LLEG_JOINT2",
                                         "LLEG_JOINT3", "LLEG_JOINT4", "LLEG_JOINT5"};
auto comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0);
comTask->selectActiveJoints(solver(), activeJoints);
solver().addTask(comTask);
```

A position task that uses every joint except the left arm:
```cpp
std::vector<std::string> inactiveJoints = {"LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2",
                                           "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5",
                                           "LARM_JOINT6", "LARM_JOINT7"};
auto posTask = std::make_shared<mc_tasks::PositionTask>("r_wrist", robots(), 0);
solver().addTask(posTask);
// You can activate the selection after adding the taks as well
posTaks->selectUnactiveJoints(solver(), inactiveJoints);
```

And you can reset the selection as well:
```cpp
posTask->resetJointsSelector(solver());
```
