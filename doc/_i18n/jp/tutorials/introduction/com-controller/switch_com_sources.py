def switch_com_target(self):
    # self.comZeroは、質量中心タスクのリセット後、
    # reset関数内で以下のように取得される:
    # self.comZero = self.comTask.com()
        self.comTask.com(self.comZero - eigen.Vector3d(0, 0, 0.2))
    else:
        self.comTask.com(self.comZero)
    self.comDown = not self.comDown
