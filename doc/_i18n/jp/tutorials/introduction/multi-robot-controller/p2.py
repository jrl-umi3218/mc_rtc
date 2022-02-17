# switch_phase(self)を編集する
if self.phase == APPROACH and self.handTask.eval().norm() < 0.05 and self.handTask.speed().norm() < 1e-4:
  # 新しい接触面を追加する
  contacts = self.qpsolver.contacts()
  contacts.append(mc_rbdyn.Contact(self.robots(), 0, 1, "RightGripper", "Handle"))
  self.qpsolver.setContacts(contacts)
  # 面変換タスクを削除する
  self.qpsolver.removeTask(self.handTask)
  # ロボットの現在の姿勢を維持する
  self.postureTask.reset()
  self.comTask.reset()
  # 新しいハンドル位置を目標とする
  self.doorPosture.target({"handle": {-1.0}})
  # フェーズを切り替える
  self.phase = HANDLE
