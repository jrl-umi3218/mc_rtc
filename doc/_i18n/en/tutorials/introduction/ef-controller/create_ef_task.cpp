// Include the EF task header (header)
#include <mc_tasks/EndEffectorTask.h>
// In the class private members (header)
std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
// In the constructor, create the task and add it to the problem
efTask = std::make_shared<mc_tasks::EndEffectorTask>("l_wrist", robots(), 0, 5.0, 500.0);
solver().addTask(efTask);
// In the reset function, reset the task to the current EF position
efTask->reset();
