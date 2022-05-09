// MyFirstController::switch_phase()を編集する
if(phase == APPROACH && handTask->eval().norm < 0.05 && handTask->speed().norm() < 1e-4)
{
  // 新しい接触面を追加する
  addContact({robot().name(), "door", "RightGripper", "Handle"});
  // 面変換タスクを削除する
  solver().removeTask(handTask);
  // ロボットの現在の姿勢を維持する
  postureTask->reset();
  comTask->reset();
  // 新しいハンドル位置を目標とする
  doorPosture->target({{"handle", {-1.0}}});
  // フェーズを切り替える
  phase = HANDLE;
}
