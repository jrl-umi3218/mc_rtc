def switch_com_target(self):
    # self.comZero is obtained by doing:
    # self.comZero = self.comTask.com()
    # in the reset function after doing the CoM task reset
    if self.comDown:
        self.comTask.com(self.comZero - eigen.Vector3d(0, 0, 0.2))
    else:
        self.comTask.com(self.comZero)
    self.comDown = not self.comDown
