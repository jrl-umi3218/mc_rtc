void MyFirstController::switch_com_target()
{
  // comZeroは、reset関数内で
  // 以下のように取得される:
  // comZero = comTask->com();
  if(comDown)
  {
    comTask->com(comZero - Eigen::Vector3d{0, 0, 0.2});
  }
  else
  {
    comTask->com(comZero);
  }
  comDown = !comDown;
}
