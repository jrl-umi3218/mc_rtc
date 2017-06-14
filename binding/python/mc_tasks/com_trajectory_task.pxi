cdef class _CoMTrajectoryTask(MetaTask):
  def __cinit__(self):
    self.ttg_base = self.mt_base = NULL
  def refVel(self, eigen.VectorXd refVel):
    assert(self.ttg_base)
    self.ttg_base.refVel(refVel.impl)
  def refAccel(self, eigen.VectorXd refAccel):
    assert(self.ttg_base)
    self.ttg_base.refAccel(refAccel.impl)
  def stiffness(self, stiff = None):
    assert(self.ttg_base)
    if stiff is None:
      return self.ttg_base.stiffness()
    else:
      self.ttg_base.stiffness(stiff)
  def setGains(self, stiffness, damping):
    assert(self.ttg_base)
    self.ttg_base.setGains(stiffness, damping)
  def damping(self):
    assert(self.ttg_base)
    return self.ttg_base.damping()
  def weight(self, w = None):
    assert(self.ttg_base)
    if w is None:
      return self.ttg_base.weight()
    else:
      self.ttg_base.weight(w)
