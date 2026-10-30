// Illustrative: replace my_sdk with the calls of your robot's SDK
RobotDriverMyRobot::RobotDriverMyRobot(const std::string & ip, uint16_t port, const std::string & config_path)
{
  // Throwing here is fine: the error is reported by uri and this robot is skipped
  robot_ = std::make_unique<my_sdk::Robot>(ip);
}

void RobotDriverMyRobot::sync()
{
  // Blocks until the next state: this call paces the control loop
  state_ = robot_->waitForNextState();
}

std::vector<double> RobotDriverMyRobot::getActualQ()
{
  return {state_.q.begin(), state_.q.end()};
}

std::vector<double> RobotDriverMyRobot::getJointTorques()
{
  return {state_.tau.begin(), state_.tau.end()};
}

void RobotDriverMyRobot::servoJ(const std::vector<double> & q)
{
  // Called every cycle with the latest command from mc_rtc
  robot_->sendJointPositions(q);
}
