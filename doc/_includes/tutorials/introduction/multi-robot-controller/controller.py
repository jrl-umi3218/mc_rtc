import eigen
import math
import mc_control
import mc_rbdyn
import mc_rtc
import mc_solver
import mc_tasks
import sva

APPROACH = 0
HANDLE = 1
OPEN = 2

class MyFirstController(mc_control.MCPythonController):
    def __init__(self, rm, dt):
        self.qpsolver.addConstraintSet(self.dynamicsConstraint)
        self.qpsolver.addConstraintSet(self.contactConstraint)
        self.qpsolver.addTask(self.postureTask)
        self.qpsolver.setContacts([
          mc_rbdyn.Contact(self.robots(), 0, 2, "LeftFoot", "AllGround"),
          mc_rbdyn.Contact(self.robots(), 0, 2, "RightFoot", "AllGround")
        ])
        self.comTask = mc_tasks.CoMTask(self.robots(), 0, 10.0, 1000.0)
        self.qpsolver.addTask(self.comTask)
        self.postureTask.stiffness(1)
        self.phase = APPROACH
    def run_callback(self):
        self.switch_phase()
        return True
    def reset_callback(self, data):
        self.comTask.reset()
        self.robots().robot(1).posW(sva.PTransformd(sva.RotZ(math.pi), eigen.Vector3d(0.7, 0.5, 0)))
        self.doorKinematics = mc_solver.KinematicsConstraint(self.robots(), 1, self.qpsolver.timeStep)
        self.qpsolver.addConstraintSet(self.doorKinematics)
        self.doorPosture = mc_tasks.PostureTask(self.qpsolver, 1, 5.0, 1000.0)
        self.qpsolver.addTask(self.doorPosture)
        self.handTask = mc_tasks.SurfaceTransformTask("RightGripper", self.robots(), 0, 5.0, 1000.0)
        self.qpsolver.addTask(self.handTask)
        self.handTask.target(sva.PTransformd(eigen.Vector3d(0, 0, -0.025)) * self.robots().robot(1).surfacePose("Handle"))
    def switch_phase(self):
      if self.phase == APPROACH and self.handTask.eval().norm() < 0.05 and self.handTask.speed().norm() < 1e-4:
        # Add a new contact
        contacts = self.qpsolver.contacts()
        contacts.append(mc_rbdyn.Contact(self.robots(), 0, 1, "RightGripper", "Handle"))
        self.qpsolver.setContacts(contacts)
        # Remove the surface transform task
        self.qpsolver.removeTask(self.handTask)
        # Keep the robot in its current posture
        self.postureTask.reset()
        self.comTask.reset()
        # Target new handle position
        self.doorPosture.target({"handle": [-1.0]})
        # Switch phase
        self.phase = HANDLE
      elif self.phase == HANDLE and self.doorPosture.eval().norm() < 0.01:
        # Update door opening target
        self.doorPosture.target({"door": [0.5]})
        # Switch phase
        self.phase = OPEN
    @staticmethod
    def create(robot, dt):
        door = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH + "/../mc_int_obj_description", "door")
        ground = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH, "ground")
        return MyFirstController([robot, door, ground], dt)
