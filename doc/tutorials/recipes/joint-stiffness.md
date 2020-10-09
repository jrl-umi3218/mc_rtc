---
layout: tutorials
toc: true
---

This facility only applies to the `PostureTask`. It is akin to the [joint selection](joint-select.html) and the [dimensional weight](dim-weight.html) features of other tasks.

It is implemented by the `JointStiffness` specification. Alternatively, one can use `JointGains` to specify a damping that is not the critical one.

It is interesting when we want some joints to better track the reference angles without disturbing the rest of the tasks:

- In some cases we want to control some angles via direct joint angle reference, such as when performing joint parameters identification
- In others, we have tasks that do not control some joints (i.e. when using a JointSelector): if we want those joints to remain at the same angle, it might be necessary to increase their stiffness

## How to use it?

Let's first create a `PostureTask` with weight 5 and stiffness 1. Then let's select a few joints of the right arm, and give them a stiffness of 4.

```cpp
{% raw %}
auto postureTask = std::make_shared<mc_tasks::PostureTask>(solver(), 0, 1, 5);
std::vector<tasks::qp::JointStiffness> stiffnesses = {{"RARM_JOINT3", 4.},
                                                      {"RARM_JOINT4", 4.}
                                                      {"RARM_JOINT5", 4.}
                                                      {"RARM_JOINT6", 4.}};

postureTask.jointsStiffness(solver(), stiffnesses);
{% endraw %}
```

In this case, all the regular stiffnesses are set to 2 and all the selected joints' dampings are set to 4.

If you want to set those dampings to another, user-defined value, you will need to use `JointGains`. Note that `JointGains` can be constructed with just a stiffness, in which case it will default to critical damping. This time let's set our stiffness to 4., but our damping to 10.

```cpp
{% raw %}
std::vector<tasks::qp::JointGains> gains = {{"RARM_JOINT3", 4., 10.},
                                            {"RARM_JOINT4", 4., 10.}
                                            {"RARM_JOINT5", 4., 10.}
                                            {"RARM_JOINT6", 4., 10.}};

postureTask.jointsGains(solver(), gain);
{% endraw %}
```
