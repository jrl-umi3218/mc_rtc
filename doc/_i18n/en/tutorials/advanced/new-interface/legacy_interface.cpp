// Sketch of a dedicated interface
mc_control::MCGlobalController controller("/path/to/mc_rtc.yaml");
auto & robot = controller.robot();

// 1. Initialize the controller from the real robot configuration
controller.init(readJointPositions());

// 2. Real-time loop, paced by the robot
while(running)
{
  waitForNextRobotState();

  // 3. Feed mc_rtc with the sensors
  controller.setEncoderValues(readJointPositions());
  controller.setJointTorques(readJointTorques());
  controller.setSensorOrientations({{"Accelerometer", readImuOrientation()}});

  // 4. Run one iteration of the controller
  if(controller.run())
  {
    // 5. Send the result to the robot
    sendJointPositions(robot.mbc().q);
  }
}
