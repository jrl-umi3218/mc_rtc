else if(phase == HANDLE && doorPosture->eval().norm() < 0.01)
{
  // ドア開の目標を更新する
  doorPosture->target({{"door", {0.5}}});
  // Switch phase
  phase = OPEN;
}
