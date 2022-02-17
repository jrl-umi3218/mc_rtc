else if(phase == HANDLE && doorPosture->eval().norm() < 0.01)
{
  // Update door opening target
  doorPosture->target({{"door", {0.5}}});
  // Switch phase
  phase = OPEN;
}
