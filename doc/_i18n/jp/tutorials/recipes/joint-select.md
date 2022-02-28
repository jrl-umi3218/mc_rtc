特定のタスク（下半身のみを使用して姿勢を安定させるタスクなど）を実行する際に、関節のサブセットを指定できます。

`MetaTask`インターフェイスでは、一連の3つの関数によってこの機能が実装されています。

```cpp
void selectActiveJoints(mc_solver::QPSolver & solver,
                        const std::vector<std::string> & activeJointsName);

void selectUnactiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & unactiveJointsName);

void resetJointsSelector(mc_solver::QPSolver & solver);
```

## 例

脚のみをシミュレートする質量中心タスク:
```cpp
std::vector<std::string> activeJoints = {"RLEG_JOINT0", "RLEG_JOINT1", "RLEG_JOINT2",
                                         "RLEG_JOINT3", "RLEG_JOINT4", "RLEG_JOINT5",
                                         "LLEG_JOINT0", "LLEG_JOINT1", "LLEG_JOINT2",
                                         "LLEG_JOINT3", "LLEG_JOINT4", "LLEG_JOINT5"};
auto comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0);
comTask->selectActiveJoints(solver(), activeJoints);
solver().addTask(comTask);
```

左腕以外のすべての関節を使用する姿勢制御タスク:
```cpp
std::vector<std::string> inactiveJoints = {"LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2",
                                           "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5",
                                           "LARM_JOINT6", "LARM_JOINT7"};
auto posTask = std::make_shared<mc_tasks::PositionTask>("r_wrist", robots(), 0);
solver().addTask(posTask);
// タスクを追加した後でも同様に選択を有効化できます
posTaks->selectUnactiveJoints(solver(), inactiveJoints);
```

選択をリセットすることもできます。
```cpp
posTask->resetJointsSelector(solver());
```
