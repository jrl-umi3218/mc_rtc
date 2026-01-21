// ヘッダー内
std::vector<std::string> urJoints_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
// reset関数内
void DualArmController::reset(const mc_control::ControllerResetData & reset_data)
{
  // ...
  urEndEffectorTask_->selectUnactiveJoints(solver(), urJoints_);
  // ...
}
// run関数内
bool DualArmController::run()
{
  if(phase_ == IDLE)
  {
    postureTask->target({{"elbow_joint", {-M_PI / 2}}, {"wrist_2_joint", {M_PI / 2}}});
    phase_ = STARTED;
  }
  // ...
}
