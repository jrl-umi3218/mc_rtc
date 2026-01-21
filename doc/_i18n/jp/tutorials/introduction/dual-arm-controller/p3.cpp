// ヘッダー内
enum ControllerState
{
  GO = 0,
  RETURN
};
// コントローラーのプライベートプロパティ
ControllerState urState_ = RETURN;
ControllerState kinovaState_ = RETURN;
// 2つのサポート関数を追加
void DualArmController::runUr()
{
  if(urState_ == GO && urEndEffectorTask_->eval().norm() < 0.05 && urEndEffectorTask_->speed().norm() < 0.01)
  {
    urState_ = RETURN;
    urEndEffectorTask_->add_ef_pose({Eigen::Vector3d(0.0, -0.5, 0.0)});
  }
  else if(urState_ == RETURN && urEndEffectorTask_->eval().norm() < 0.01 && urEndEffectorTask_->speed().norm() < 0.05)
  {
    urState_ = GO;
    urEndEffectorTask_->add_ef_pose({Eigen::Vector3d(0.0, 0.5, 0.0)});
  }
}

void DualArmController::runKinova()
{
  if(kinovaState_ == GO && kinovaPostureTask_->eval().norm() < 0.01 && kinovaPostureTask_->speed().norm() < 0.01)
  {
    kinovaState_ = RETURN;
    kinovaPostureTask_->target({{"joint_2", {0.0}}});
  }
  else if(kinovaState_ == RETURN && kinovaPostureTask_->eval().norm() < 0.01
          && kinovaPostureTask_->speed().norm() < 0.01)
  {
    kinovaState_ = GO;
    kinovaPostureTask_->target({{"joint_2", {-M_PI / 4}}});
  }
}
// run関数内
bool DualArmController::run()
{
  // ...
  else if(phase_ == MOVE)
  {
    runUr();
    runKinova();
  }
  return mc_control::MCController::run();
}
