elif self.phase == HANDLE and self.doorPosture.eval().norm() < 0.01:
  # ドア開の目標を更新する
  self.doorPosture.target({"door": [0.5]})
  # フェーズを切り替える
  self.phase = OPEN
