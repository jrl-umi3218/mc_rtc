import mc_control
import mc_rbdyn
import mc_rtc

class MyFirstController(mc_control.MCPythonController):
    def __init__(self, rm, dt):
        self.qpsolver.addConstraintSet(self.kinematicsConstraint)
        self.qpsolver.addConstraintSet(self.contactConstraint)
        self.qpsolver.addTask(self.postureTask)
        self.qpsolver.setContacts([])
        self.jointIndex = self.robot().jointIndexByName("NECK_Y")
        self.goingLeft = True
    def run_callback(self):
        if abs(self.postureTask.posture()[self.jointIndex][0] - self.robot().mbc.q[self.jointIndex][0]) < 0.05:
            self.switch_target()
        return True
    def reset_callback(self, data):
        pass
    def switch_target(self):
        if self.goingLeft:
            self.postureTask.target({"NECK_Y": self.robot().qu[self.jointIndex]})
        else:
            self.postureTask.target({"NECK_Y": self.robot().ql[self.jointIndex]})
        self.goingLeft = not self.goingLeft
    @staticmethod
    def create(robot, dt):
        env = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH, "ground")
        return MyFirstController([robot,env], dt)
