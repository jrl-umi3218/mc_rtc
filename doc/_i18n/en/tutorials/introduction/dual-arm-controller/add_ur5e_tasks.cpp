// In the header
std::shared_ptr<mc_tasks::EndEffectorTask> urEndEffectorTask_;
// In the reset function
urEndEffectorTask_ = std::make_shared<mc_tasks::EndEffectorTask>(urBody, robots(), 0);
urEndEffectorTask_->positionTask->stiffness(1);
urEndEffectorTask_->orientationTask->stiffness(1);
solver().addTask(urEndEffectorTask_);
