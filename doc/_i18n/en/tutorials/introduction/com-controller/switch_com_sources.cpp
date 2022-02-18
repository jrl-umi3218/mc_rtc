void MyFirstController::switch_com_target()
{
  // comZero is obtained by doing:
  // comZero = comTask->com();
  // in the reset function
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
