// 専用インターフェイスのスケッチ
mc_control::MCGlobalController controller("/path/to/mc_rtc.yaml");
auto & robot = controller.robot();

// 1. 実ロボットの状態からコントローラーを初期化する
controller.init(readJointPositions());

// 2. ロボットに同期したリアルタイムループ
while(running)
{
  waitForNextRobotState();

  // 3. mc_rtcにセンサー値を渡す
  controller.setEncoderValues(readJointPositions());
  controller.setJointTorques(readJointTorques());
  controller.setSensorOrientations({{"Accelerometer", readImuOrientation()}});

  // 4. コントローラーを1回実行する
  if(controller.run())
  {
    // 5. 結果をロボットに送る
    sendJointPositions(robot.mbc().q);
  }
}
