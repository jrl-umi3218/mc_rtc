import mc_control
import mc_rbdyn
import math
import mc_tasks
import sva
import eigen


class ControllerPhase:
    IDLE = 0
    STARTED = 1
    MOVE = 2


class ControllerState:
    GO = 0
    RETURN = 1


class DualArmController(mc_control.MCPythonController):
    def __init__(self, rm, dt):
        self.qpsolver.addConstraintSet(self.contactConstraint)
        self.qpsolver.addConstraintSet(self.kinematicsConstraint)
        self.qpsolver.addConstraintSet(self.selfCollisionConstraint)
        iDist, sDist, damping = 0.1, 0.05, 0.1
        self.addCollisions(
            "ur5e",
            "kinova_default",
            [mc_rbdyn.Collision("*", "*", iDist, sDist, damping)],
        )
        self.postureTask.stiffness(1)
        self.postureTask.weight(1)
        self.qpsolver.addTask(self.postureTask)
        self._phase = ControllerPhase.IDLE
        self._ur_state = ControllerState.RETURN
        self._kinova_state = ControllerState.RETURN
        self._urEndEffectorTask = None
        self._kinovaPostureTask = None
        ur_joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self._ur_joints = [joint.encode("utf-8") for joint in ur_joints]

    def run_callback(self):
        if self._phase == ControllerPhase.IDLE:
            self.postureTask.target(
                {
                    "elbow_joint".encode("utf-8"): [-math.pi / 2],
                    "wrist_2_joint".encode("utf-8"): [math.pi / 2],
                }
            )
            self._phase = ControllerPhase.STARTED
        elif (
            self._phase == ControllerPhase.STARTED
            and self.postureTask.eval().norm() < 0.01
            and self.postureTask.speed().norm() < 0.01
        ):
            self._phase = ControllerPhase.MOVE
            self._urEndEffectorTask.selectActiveJoints(self.qpsolver, self._ur_joints)
        elif self._phase == ControllerPhase.STARTED:
            self._urEndEffectorTask.reset()
        elif self._phase == ControllerPhase.MOVE:
            self._run_ur5e()
            self._run_kinova()
        return True

    def reset_callback(self, data):
        self.postureTask.reset()
        self._urEndEffectorTask = mc_tasks.EndEffectorTask(
            "wrist_3_link", self.robots(), 0
        )
        self._urEndEffectorTask.positionTask.stiffness(1)
        self._urEndEffectorTask.orientationTask.stiffness(1)
        self._urEndEffectorTask.selectUnactiveJoints(
            self.qpsolver, self._ur_joints
        )
        self._kinovaPostureTask = mc_tasks.PostureTask(
            self.qpsolver, 1, 5.0, 1000.0
        )
        self.qpsolver.addTask(self._urEndEffectorTask)
        self.qpsolver.addTask(self._kinovaPostureTask)
        self.robots().robot(1).posW(
            sva.PTransformd(sva.RotZ(0), eigen.Vector3d(0.7, 0.5, 0))
        )

    @staticmethod
    def create(robot, dt):
        kinova = mc_rbdyn.get_robot_module(
            "env", "/usr/local/share/mc_kinova", "kinova_default"
        )
        return DualArmController([robot, kinova], dt)

    def _run_kinova(self):
        if (
            self._kinova_state == ControllerState.GO
            and self._kinovaPostureTask.eval().norm() < 0.01
            and self._kinovaPostureTask.speed().norm() < 0.01
        ):
            self._kinova_state = ControllerState.RETURN
            self._kinovaPostureTask.target(
                {
                    "joint_2".encode("utf-8"): [0.0],
                }
            )
        elif (
            self._kinova_state == ControllerState.RETURN
            and self._kinovaPostureTask.eval().norm() < 0.01
            and self._kinovaPostureTask.speed().norm() < 0.01
        ):
            self._kinova_state = ControllerState.GO
            self._kinovaPostureTask.target(
                {
                    "joint_2".encode("utf-8"): [-math.pi / 4],
                }
            )

    def _run_ur5e(self):
        if (
            self._ur_state == ControllerState.GO
            and self._urEndEffectorTask.positionTask.eval().norm() < 0.05
            and self._urEndEffectorTask.positionTask.speed().norm() < 0.05
        ):
            self._ur_state = ControllerState.RETURN
            self._urEndEffectorTask.add_ef_pose(sva.PTransformd(
                sva.RotZ(0), eigen.Vector3d(0.0, -0.5, 0.0)
            ))
        elif (
            self._ur_state == ControllerState.RETURN
            and self._urEndEffectorTask.positionTask.eval().norm() < 0.05
            and self._urEndEffectorTask.positionTask.speed().norm() < 0.05
        ):
            self._ur_state = ControllerState.GO
            self._urEndEffectorTask.add_ef_pose(sva.PTransformd(
                sva.RotZ(0), eigen.Vector3d(0.0, 0.5, 0.0)
            ))
