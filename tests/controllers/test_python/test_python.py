import eigen
import sva
import mc_rbdyn
import mc_control
import mc_rtc
import mc_tasks
import time

from functools import partial

class TestPythonController(mc_control.MCPythonController):
  def __init__(self, rm, dt):
    self.qpsolver.addConstraintSet(self.dynamicsConstraint)
    self.qpsolver.addConstraintSet(self.contactConstraint)
    self.qpsolver.addTask(self.postureTask)
    self.positionTask = mc_tasks.PositionTask("RARM_LINK6", self.robots(), 0)
    self.qpsolver.addTask(self.positionTask)
    self.qpsolver.setContacts([
          mc_rbdyn.Contact(self.robots(), "LFullSole", "AllGround"),
          mc_rbdyn.Contact(self.robots(), "RFullSole", "AllGround")
        ])
    self.hj1_name = "HEAD_JOINT1"
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
    return True
  def get_pt(self):
    return self.robot().mbc.bodyPosW[0]
  def reset_callback(self, reset_data):
    self.positionTask.reset()
    self.positionTask.position(self.positionTask.position() + eigen.Vector3d(0.1, 0, 0))
    self.set_joint_pos(self.hj1_name, self.hj1_q)
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
  @staticmethod
  def create(robot, dt):
    env = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH, "ground")
    return TestPythonController([robot,env], dt)
