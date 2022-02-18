# コンストラクター内
self.qpsolver.setContacts([
  mc_rbdyn.Contact(self.robots(), 0, 1, "LeftFoot", "AllGround"),
  mc_rbdyn.Contact(self.robots(), 0, 1, "RightFoot", "AllGround")
])
