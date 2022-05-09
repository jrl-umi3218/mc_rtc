import eigen
import mc_control
import mc_rbdyn
import mc_rtc
import mc_tasks

class MyFirstController(mc_control.MCPythonController):
    def __init__(self, rm, dt):
        self.qpsolver.addConstraintSet(self.dynamicsConstraint)
        self.qpsolver.addConstraintSet(self.contactConstraint)
        self.qpsolver.addTask(self.postureTask)
        self.addContact(self.robot().name(), "ground", "LeftFoot", "AllGround")
        self.addContact(self.robot().name(), "ground", "RightFoot", "AllGround")
        self.comTask = mc_tasks.CoMTask(self.robots(), 0, 10.0, 1000.0)
        self.qpsolver.addTask(self.comTask)
        self.postureTask.stiffness(1)
        self.jointIndex = self.robot().jointIndexByName("NECK_Y")
        self.goingLeft = True
        self.comDown = True
        self.comZero = eigen.Vector3d.Zero()
    def run_callback(self):
        if abs(self.postureTask.posture()[self.jointIndex][0] - self.robot().mbc.q[self.jointIndex][0]) < 0.05:
            self.switch_target()
        if self.comTask.eval().norm() < 0.01:
            self.switch_com_target()
        return True
    def reset_callback(self, data):
        self.comTask.reset()
        self.comZero = self.comTask.com()
    def switch_target(self):
        if self.goingLeft:
            self.postureTask.target({"NECK_Y": self.robot().qu[self.jointIndex]})
        else:
            self.postureTask.target({"NECK_Y": self.robot().ql[self.jointIndex]})
        self.goingLeft = not self.goingLeft
    def switch_com_target(self):
        if self.comDown:
            self.comTask.com(self.comZero - eigen.Vector3d(0, 0, 0.2))
        else:
            self.comTask.com(self.comZero)
        self.comDown = not self.comDown
    @staticmethod
    def create(robot, dt):
        env = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH, "ground")
        return MyFirstController([robot,env], dt)
