#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

import mc_tasks
import mc_rbdyn
import eigen as e

from nose import with_setup


class TestMCTasks():

  @classmethod
  def setup_class(self):
    self.robots = mc_rbdyn.Robots()
    mc_rbdyn.RobotLoader.clear()
    mc_rbdyn.RobotLoader.update_robot_module_path(["$<TARGET_FILE_DIR:jvrc1>"])
    self.rm = mc_rbdyn.get_robot_module("JVRC1")
    self.robots.load(self.rm)

  @classmethod
  def teardown_class(self):
    pass

  def test_comTask(self):
    comTask1 = mc_tasks.CoMTask(self.robots, 0)
    comTask2 = mc_tasks.CoMTask(self.robots, 0, 2.0, 500)
    comTask3 = mc_tasks.CoMTask(self.robots, 0, weight=500.0, stiffness=2.0)

    assert(comTask1.stiffness() == comTask2.stiffness())
    assert(comTask1.stiffness() == comTask3.stiffness())
    assert(comTask1.weight() == comTask2.weight())
    assert(comTask1.weight() == comTask3.weight())

  def test_posTask(self):
    posTask1 = mc_tasks.PositionTask("r_wrist", self.robots, 0)
    posTask2 = mc_tasks.PositionTask("r_wrist", self.robots, 0, 2.0, 500)
    posTask3 = mc_tasks.PositionTask("r_wrist", self.robots, 0, weight=500.0, stiffness=2.0)

    assert(posTask1.stiffness() == posTask2.stiffness())
    assert(posTask1.stiffness() == posTask3.stiffness())
    assert(posTask1.weight() == posTask2.weight())
    assert(posTask1.weight() == posTask3.weight())

  def test_oriTask(self):
    oriTask1 = mc_tasks.OrientationTask("r_wrist", self.robots, 0)
    oriTask2 = mc_tasks.OrientationTask("r_wrist", self.robots, 0, 2.0, 500)
    oriTask3 = mc_tasks.OrientationTask("r_wrist", self.robots, 0, weight=500.0, stiffness=2.0)

    assert(oriTask1.stiffness() == oriTask2.stiffness())
    assert(oriTask1.stiffness() == oriTask3.stiffness())
    assert(oriTask1.weight() == oriTask2.weight())
    assert(oriTask1.weight() == oriTask3.weight())

  def test_vecOriTask(self):
    vecTask1 = mc_tasks.VectorOrientationTask("r_wrist", e.Vector3d(0., 0., 1.), e.Vector3d(1., 0., 0.), self.robots, 0)
    vecTask2 = mc_tasks.VectorOrientationTask("r_wrist", e.Vector3d(0., 0., 1.), e.Vector3d(1., 0., 0.), self.robots, 0, 2.0, 500)
    vecTask3 = mc_tasks.VectorOrientationTask("r_wrist", e.Vector3d(0., 0., 1.), e.Vector3d(1., 0., 0.), self.robots, 0, weight=500.0, stiffness=2.0)

    assert(vecTask1.stiffness() == vecTask2.stiffness())
    assert(vecTask1.stiffness() == vecTask3.stiffness())
    assert(vecTask1.weight() == vecTask2.weight())
    assert(vecTask1.weight() == vecTask3.weight())

  def test_EFTask(self):
    efTask1 = mc_tasks.EndEffectorTask("r_wrist", self.robots, 0)
    efTask2 = mc_tasks.EndEffectorTask("r_wrist", self.robots, 0, 2.0, 1000)
    efTask3 = mc_tasks.EndEffectorTask("r_wrist", self.robots, 0, weight=1000.0, stiffness=2.0)

    assert(efTask1.positionTask.stiffness() == efTask2.positionTask.stiffness())
    assert(efTask1.positionTask.stiffness() == efTask3.positionTask.stiffness())
    assert(efTask1.positionTask.weight() == efTask2.positionTask.weight())
    assert(efTask1.positionTask.weight() == efTask3.positionTask.weight())
    assert(efTask1.orientationTask.stiffness() == efTask2.orientationTask.stiffness())
    assert(efTask1.orientationTask.stiffness() == efTask3.orientationTask.stiffness())
    assert(efTask1.orientationTask.weight() == efTask2.orientationTask.weight())
    assert(efTask1.orientationTask.weight() == efTask3.orientationTask.weight())

  def test_RelEFTask(self):
    relEfTask1 = mc_tasks.RelativeEndEffectorTask("r_wrist", self.robots, 0)
    relEfTask2 = mc_tasks.RelativeEndEffectorTask("r_wrist", self.robots, 0, "", 2.0, 1000)
    relEfTask3 = mc_tasks.RelativeEndEffectorTask("r_wrist", self.robots, 0, weight=1000.0, stiffness=2.0, relBodyName="")

    assert(relEfTask1.positionTask.stiffness() == relEfTask2.positionTask.stiffness())
    assert(relEfTask1.positionTask.stiffness() == relEfTask3.positionTask.stiffness())
    assert(relEfTask1.positionTask.weight() == relEfTask2.positionTask.weight())
    assert(relEfTask1.positionTask.weight() == relEfTask3.positionTask.weight())
    assert(relEfTask1.orientationTask.stiffness() == relEfTask2.orientationTask.stiffness())
    assert(relEfTask1.orientationTask.stiffness() == relEfTask3.orientationTask.stiffness())
    assert(relEfTask1.orientationTask.weight() == relEfTask2.orientationTask.weight())
    assert(relEfTask1.orientationTask.weight() == relEfTask3.orientationTask.weight())

  def test_complianceTask(self):
    compTask1 = mc_tasks.force.ComplianceTask(self.robots, 0, "R_WRIST_Y_S", 0.005)
    compTask2 = mc_tasks.force.ComplianceTask(self.robots, 0, "R_WRIST_Y_S", 0.005, 2.0, 1000, 3., 1., mc_tasks.force.ComplianceTask.defaultFGain, mc_tasks.force.ComplianceTask.defaultTGain)
    compTask3 = mc_tasks.force.ComplianceTask(self.robots, 0, "R_WRIST_Y_S", 0.005, weight=1000.0, stiffness=2.0, forceGain = mc_tasks.force.ComplianceTask.defaultFGain, torqueThresh = 1., torqueGain = mc_tasks.force.ComplianceTask.defaultTGain, forceThresh = 3.)

  def test_surfaceTransformTask(self):
    surfaceTask1 = mc_tasks.SurfaceTransformTask("LeftFoot", self.robots, 0)
    surfaceTask2 = mc_tasks.SurfaceTransformTask("LeftFoot", self.robots, 0, 5.0, 1000.0)

  def test_admittanceTask(self):
    admTask1 = mc_tasks.force.AdmittanceTask("LeftFoot", self.robots, 0)
    admTask2 = mc_tasks.force.AdmittanceTask("LeftFoot", self.robots, 0, 5.0, 1000.0)
