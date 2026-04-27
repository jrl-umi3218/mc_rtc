// ヘッダー内
#include <mc_tasks/EndEffectorTask.h>

struct DualArmController_DLLAPI DualArmController : public mc_control::MCController
{
  // ...
  std::shared_ptr<mc_tasks::EndEffectorTask> urEndEffectorTask_;
  // ...
}

// reset関数内
std::string urBody = "tool0";
urEndEffectorTask_ = std::make_shared<mc_tasks::EndEffectorTask>(urBody, robots(), 0, 1);
solver().addTask(urEndEffectorTask_);
