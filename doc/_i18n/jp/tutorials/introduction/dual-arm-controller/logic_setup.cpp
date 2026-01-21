// ヘッダー内
enum ControllerPhase { IDLE = 0, STARTED, MOVE };
// コントローラーのプライベートプロパティ
ControllerPhase phase = IDLE;
// run関数内
bool DualArmController::run()
{
  if (phase_ == IDLE && 0 /** この条件は後で記述する */) {
    /** STARTEDフェーズをセットアップする */
    phase_ = STARTED;
  } else if (phase_ == STARTED && 0 /** この条件は後で記述する */) {
    /** MOVEフェーズをセットアップする */
    phase_ = MOVE;
  }
  else if (phase_ == MOVE && 0 /** この条件は後で記述する */) {
  
  }
  return mc_control::MCController::run();
}
