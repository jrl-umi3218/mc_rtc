// In the header
#include <mc_tasks/EndEffectorTask.h>

struct DualArmController_DLLAPI DualArmController : public mc_control::MCController
{
  // ...
  std::shared_ptr<mc_tasks::EndEffectorTask> urEndEffectorTask_;
  // ...
}

// In the reset function
std::string urBody = "wrist_3_link";
urEndEffectorTask_ = std::make_shared<mc_tasks::EndEffectorTask>(urBody, robots(), 0);
urEndEffectorTask_->positionTask->stiffness(1);
urEndEffectorTask_->orientationTask->stiffness(1);
solver().addTask(urEndEffectorTask_);
