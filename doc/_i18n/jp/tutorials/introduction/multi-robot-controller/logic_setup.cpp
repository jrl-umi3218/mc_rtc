// ヘッダー内
enum DoorPhase
{
      APPROACH = 0,
      HANDLE,
      OPEN
};
// コントローラーのプライベートプロパティ
DoorPhase phase = APPROACH;
// コントローラーの新しいメソッド
void switch_phase()
{
  if(phase == APPROACH && 0 /** この条件は後で記述する */)
  {
    /** HANDLEフェーズをセットアップする */
    phase = HANDLE;
  }
  else if(phase == HANDLE && 0 /** この条件は後で記述する */)
  {
    /** OPENフェーズをセットアップする */
    phase = OPEN;
  }
}
// run関数内でこれを呼び出す
bool MyFirstController::run()
{
  switch_phase();
  return mc_control::MCController::run();
}
