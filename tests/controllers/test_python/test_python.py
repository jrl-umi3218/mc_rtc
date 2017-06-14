import eigen
import mc_rbdyn
import mc_control
import mc_rtc
import mc_tasks
import time

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
  def run_callback(self):
    hj1_q = self.robot().mbc.q[self.hj1_index][0]
    return True
  def reset_callback(self, reset_data):
    self.positionTask.reset()
    self.positionTask.position(self.positionTask.position() + eigen.Vector3d(0.1, 0, 0))
    self.set_joint_pos(self.hj1_name, self.hj1_q)
  def log_header_callback(self):
    return ";PythonCustomLogValue;ASecondValue"
  def log_data_callback(self):
    return ";{0};42.0".format(self.robot().mbc.q[self.hj1_index][0])
  @staticmethod
  def create(robot, dt):
    env = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH, "ground")
    return TestPythonController([robot,env], dt)
