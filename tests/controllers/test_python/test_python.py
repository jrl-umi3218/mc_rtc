#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

import eigen
import sva
import mc_rbdyn
import mc_control
import mc_rtc
import mc_tasks
import sys
import time

from functools import partial

class TestPythonController(mc_control.MCPythonController):
  def __init__(self, rm, dt):
    self.qpsolver.addConstraintSet(self.dynamicsConstraint)
    self.qpsolver.addConstraintSet(self.contactConstraint)
    self.qpsolver.addTask(self.postureTask)
    self.positionTask = mc_tasks.PositionTask("R_WRIST_Y_S", self.robots(), 0)
    self.qpsolver.addTask(self.positionTask)
    self.qpsolver.setContacts([
          mc_rbdyn.Contact(self.robots(), "LeftFoot", "AllGround"),
          mc_rbdyn.Contact(self.robots(), "RightFoot", "AllGround")
        ])
    self.hj1_name = "NECK_P"
    self.hj1_index = self.robot().jointIndexByName(self.hj1_name)
    self.hj1_q = 0.5
    # Stuff to log
    self.v3d_data = eigen.Vector3d.Random()
    self.logger().addLogEntry("PYTHONV3D", lambda: self.v3d_data)
    self.d_data = 0.0
    self.dv_data = [self.d_data]*3
    self.theta = 0
    self.quat_data = eigen.Quaterniond(sva.RotZ(self.theta))
  def run_callback(self):
    hj1_q = self.robot().mbc.q[self.hj1_index][0]
    self.v3d_data = eigen.Vector3d.Random()
    self.d_data += 1.0
    self.dv_data = [self.d_data]*3
    self.theta += 0.005
    self.quat_data = eigen.Quaterniond(sva.RotZ(self.theta))
    assert(self.observerPipeline("FirstPipeline").success())
    if abs(self.d_data - 2.0) < 1e-6:
      self.removeAnchorFrameCallback("KinematicAnchorFrame::{}".format(self.robot().name()))
      self.addAnchorFrameCallback("KinematicAnchorFrame::{}".format(self.robot().name()), self.anchorFrameCallback)
    return True
  def get_pt(self):
    return self.robot().mbc.bodyPosW[0]
  def anchorFrameCallback(self, r):
    return sva.interpolate(r.surfacePose("LeftFoot"), r.surfacePose("RightFoot"), 0.5)
  def reset_callback(self, reset_data):
    assert(len(self.observerPipelines()) == 1)
    assert(self.hasObserverPipeline("FirstPipeline"))
    assert(not self.hasObserverPipeline("NotAPipeline"))
    self.addAnchorFrameCallback("KinematicAnchorFrame::{}".format(self.robot().name().decode()), lambda r: sva.interpolate(r.surfacePose("LeftFoot"), r.surfacePose("RightFoot"), 0.5))
    self.positionTask.reset()
    self.positionTask.position(self.positionTask.position() + eigen.Vector3d(0.1, 0, 0))
    self.logger().addLogEntry("PYTHONDOUBLE", lambda: self.d_data)
    self.logger().addLogEntry("PYTHONDOUBLEV", lambda: self.dv_data)
    self.logger().addLogEntry("PYTHONQUAT", lambda: self.quat_data)
    # Demonstrate use of partial
    self.logger().addLogEntry("PYTHONPT", partial(self.get_pt))
    # Alternative syntax for above call
    # self.logger().addLogEntry("PYTHONPT", partial(SampleController.get_pt, self))
    self.logger().addLogEntry("PYTHONFV", lambda: sva.ForceVecd(eigen.Vector6d(0, 0, 0, 0, 0, 100)) + sva.ForceVecd(eigen.Vector6d(0, 0, 0, 0, 0, 100)))
    # Not a very efficient way to log a double value
    self.logger().addLogEntry("PYTHONSTR", lambda: str(self.theta))
    # GUI similar to dummyServer
    self.gui().addElement(["Python"], mc_rtc.gui.Label("theta", lambda: self.theta))
    self.gui().addElement(["Python"], mc_rtc.gui.ArrayLabel("array", lambda: [self.theta,2*self.theta,4*self.theta]),
                                      mc_rtc.gui.ArrayLabel("array with labels", lambda: [self.theta,2*self.theta,4*self.theta], ["x", "y", "z"]))
    self.gui().addElement(["Python"], mc_rtc.gui.Button("button", lambda: sys.stdout.write("Button clicked\n")))
    self.check_ = True
    self.gui().addElement(["Python"], mc_rtc.gui.Checkbox("checkbox", lambda: self.check_, self.check))
    self.mystring_ = "test"
    self.myint_ = 0
    self.mynumber_ = 0.0
    self.gui().addElement(["Python"], mc_rtc.gui.StringInput("string input", self.mystring, self.mystring),
                                      mc_rtc.gui.IntegerInput("integer input", self.myint, self.myint),
                                      mc_rtc.gui.NumberInput("number input", self.mynumber, self.mynumber))
    self.myslider_ = 0.0
    self.gui().addElement(["Python"], mc_rtc.gui.NumberSlider("number slider", self.myslider, self.myslider, -100.0, 100.0))
    self.myarray_ = [0.,1.,2.,3.]
    self.gui().addElement(["Python"], mc_rtc.gui.ArrayInput("array input", self.myarray, self.myarray))
    self.gui().addElement(["Python"], mc_rtc.gui.ArrayInput("array input with labels", self.myarray, self.myarray, ["w", "x", "y", "z"]))
    self.mycombo_ = "a"
    self.gui().addElement(["Python"], mc_rtc.gui.ComboInput("combo input", ["a", "b", "c", "d"], self.mycombo, self.mycombo))
    self.mydatacombo_ = ""
    self.gui().addElement(["Python"], mc_rtc.gui.DataComboInput("data combo", ["robots"], self.mydatacombo, self.mydatacombo))
    self.mypoint3dro_ = [0., 1., 0.]
    self.gui().addElement(["Python"], mc_rtc.gui.Point3D("Point3D ro", self.mypoint3dro))
    self.mypoint3d_ = [0., 0., 1.]
    self.gui().addElement(["Python"], mc_rtc.gui.Point3D("Point3D", self.mypoint3d, self.mypoint3d))
    self.gui().addElement(["Python", "Form"], mc_rtc.gui.Form("Submit", lambda c: sys.stdout.write(c.dump(True) + '\n'),
                                                                mc_rtc.gui.FormCheckbox("Enabled", False, True),
                                                                mc_rtc.gui.FormIntegerInput("INT", False, 42),
                                                                mc_rtc.gui.FormNumberInput("NUMBER", False, 0.42),
                                                                mc_rtc.gui.FormStringInput("STRING", False, "a certain string"),
                                                                mc_rtc.gui.FormNumberArrayInput("ARRAY_FIXED_SIZE", False, [1,2,3]),
                                                                mc_rtc.gui.FormNumberArrayInput("ARRAY_UNBOUNDED", False),
                                                                mc_rtc.gui.FormComboInput("CHOOSE WISELY", False, ["A", "B", "C", "D"]),
                                                                mc_rtc.gui.FormDataComboInput("R0", False, ["robots"]),
                                                                mc_rtc.gui.FormDataComboInput("R0 surface", False, ["surfaces", "$R0"]),
                                                                mc_rtc.gui.FormDataComboInput("R1", False, ["robots"]),
                                                                mc_rtc.gui.FormDataComboInput("R1 surface", False, ["surfaces", "$R1"])))
    self._selfDestructId = 0
    self.gui().addElement(["Python", "Experiment"], mc_rtc.gui.Button("Self-destruct category", lambda: self.gui().removeCategory(["Python", "Experiment"])))
    self.gui().addElement(["Python", "Experiment"], mc_rtc.gui.Button("Add self-destruct button", self.addSelfDestructButton))
  def addSelfDestructButton(self):
    bname = "Self destruct {}".format(self._selfDestructId)
    self._selfDestructId += 1
    self.gui().addElement(["Python", "Experiment"], mc_rtc.gui.Button(bname, lambda: self.gui().removeElement(["Python", "Experiment"], bname)))
  def mystring(self, s = None):
    if s is None:
        return self.mystring_
    else:
        print("Change mystring to {}".format(s))
        self.mystring_ = s
  def myint(self, s = None):
    if s is None:
        return self.myint_
    else:
        print("Change myint to {}".format(s))
        self.myint_ = s
  def mynumber(self, s = None):
    if s is None:
        return self.mynumber_
    else:
        print("Change mynumber to {}".format(s))
        self.mynumber_ = s
  def myslider(self, s = None):
    if s is None:
        return self.myslider_
    else:
        print("Change myslider to {}".format(s))
        self.myslider_ = s
  def myarray(self, s = None):
    if s is None:
        return self.myarray_
    else:
        print("Change myarray to {}".format(s))
        self.myarray_ = s
  def mycombo(self, s = None):
    if s is None:
        return self.mycombo_
    else:
        print("Change mycombo to {}".format(s))
        self.mycombo_ = s
  def mydatacombo(self, s = None):
    if s is None:
        return self.mydatacombo_
    else:
        print("Change mydatacombo to {}".format(s))
        self.mydatacombo_ = s
  def mypoint3dro(self, s = None):
    if s is None:
        return self.mypoint3dro_
    else:
        print("Change mypoint3dro to {}".format(s))
        self.mypoint3dro_ = s
  def mypoint3d(self, s = None):
    if s is None:
        return self.mypoint3d_
    else:
        print("Change mypoint3d to {}".format(s))
        self.mypoint3d_ = s
  def check(self):
    self.check_ = not self.check_
    print("Change check to {}".format(self.check_))
  @staticmethod
  def create(robot, dt):
    env = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH, "ground")
    return TestPythonController([robot,env], dt)
