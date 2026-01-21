// run関数内
bool DualArmController::run() {
  // ...
  else if (phase_ == STARTED && postureTask->eval().norm() < 0.01 &&
      postureTask->speed().norm() < 0.01) {
    phase_ = MOVE;
    urEndEffectorTask_->selectActiveJoints(solver(), urJoints_);
  } else if (phase_ == STARTED) {
    urEndEffectorTask_->reset();
  }
  // ...
  return mc_control::MCController::run();
}