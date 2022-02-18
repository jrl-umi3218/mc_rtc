# resetコールバック関数内
# タスクを作成してソルバーに追加する
self.handTask = mc_tasks.SurfaceTransformTask("RightGripper", self.robots(), 0, 5.0, 1000.0)
self.qpsolver.addTask(self.handTask)
# ハンドルの位置を基準とした目標を設定する
self.handTask.target(sva.PTransformd(eigen.Vector3d(0, 0, -0.025)) * self.robots().robot(1).surfacePose("Handle"))
