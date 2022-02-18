// Include the CoM task header (header)
#include <mc_tasks/CoMTask.h>
// In the class private members (header)
std::shared_ptr<mc_tasks::CoMTask> comTask;
// In the constructor, create the task and add it to the problem
comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
solver().addTask(comTask);
// Reduce the posture task stiffness
postureTask->stiffness(1);
// In the reset function, reset the task to the current CoM
comTask->reset();
