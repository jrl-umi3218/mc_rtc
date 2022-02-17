// In the header
#include <mc_tasks/SurfaceTransformTask.h>
// In the private members
std::shared_ptr<mc_tasks::SurfaceTransformTask> handTask;
// In the reset function
// Create the task and add it to the solver
handTask = std::make_shared<mc_tasks::SurfaceTransformTask>("RightGripper", robots(), 0, 5.0, 1000.0);
solver().addTask(handTask);
// Set a target relative to the handle position
handTask->target(sva::PTransformd(Eigen::Vector3d(0, 0, -0.025)) * robots().robot(1).surfacePose("Handle"));
