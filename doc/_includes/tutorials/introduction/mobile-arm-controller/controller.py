import os
import sva
import math
import eigen
import mc_tasks
import mc_rbdyn
import mc_solver
import mc_control


class ControllerPhase:
    APPROACH = 0
    HANDLE = 1
    OPEN = 2
    DONE = 3


class MobileArmController(mc_control.MCPythonController):
    def __init__(self, rm, dt):
        self.qpsolver.addConstraintSet(self.contactConstraint)
        self.qpsolver.addConstraintSet(self.selfCollisionConstraint)
        dof = eigen.Vector6d.Zero()
        dof[0] = 1.0
        dof[1] = 1.0
        dof[5] = 1.0
        friction = mc_rbdyn.Contact.defaultFriction
        self.addContact(
            mc_control.Contact("dingo", "ground", "Base", "AllGround", friction, dof)
        )
        iDist, sDist, damping = 0.1, 0.05, 0.0
        self.addCollisions(
            "dingo",
            "door",
            [mc_rbdyn.Collision("*", "*", iDist, sDist, damping)],
        )
        # self.addCollisions(
        #     "ur5e",
        #     "door",
        #     [mc_rbdyn.Collision("*", "*", iDist, sDist, damping)],
        # )
        self._phase = ControllerPhase.APPROACH

    def run_callback(self):
        if self._phase == ControllerPhase.APPROACH and self.count < 1000:
            self.count += 1
        elif (
            self._phase == ControllerPhase.APPROACH
            and self._dingoEndEffectorTask.eval().norm() < 1e-5
            and self._dingoEndEffectorTask.speed().norm() < 1e-5
        ):
            self.qpsolver.removeTask(self.postureTask)
            self._handTask.reset()
            self.qpsolver.addTask(self._handTask)
            self._handTask.target(
                sva.PTransformd(eigen.Vector3d(0.0, 0.0, -0.05))
                * self.robots().robot(2).surfacePose("Handle")
            )
            self._phase = ControllerPhase.HANDLE
        elif (
            self._phase == ControllerPhase.HANDLE
            and self._handTask.eval().norm() < 0.1
            and self._handTask.speed().norm() < 1e-4
        ):
            self.addContact(mc_control.Contact("ur5e", "door", "Wrist", "Handle"))
            self.qpsolver.removeTask(self._handTask)
            self.postureTask.reset()
            self._doorPosture.target(
                {
                    "handle".encode("utf-8"): [-1.0],
                }
            )
            self._phase = ControllerPhase.OPEN
        elif (
            self._phase == ControllerPhase.OPEN
            and self._doorPosture.eval().norm() < 0.01
        ):
            self.qpsolver.removeTask(self._dingoEndEffectorTask)
            self._doorPosture.target(
                {
                    "door".encode("utf-8"): [math.pi / 2],
                }
            )
            self._phase = ControllerPhase.DONE
        elif (
            self._phase == ControllerPhase.DONE
            and self._doorPosture.eval().norm() < 0.01
        ):
            self.removeContact(mc_control.Contact("ur5e", "door", "Wrist", "Handle"))
            self._dingoEndEffectorTask.reset()
            self.qpsolver.addTask(self._dingoEndEffectorTask)
            self.qpsolver.addTask(self.postureTask)
        return True

    def reset_callback(self, data):
        self.doorKinematics = mc_solver.KinematicsConstraint(
            self.robots(), 2, self.qpsolver.timeStep
        )
        self.qpsolver.addConstraintSet(self.doorKinematics)
        self.robots().robot(0).posW(
            sva.PTransformd(sva.RotZ(0), eigen.Vector3d(0.0, 0.0, 0.5))
        )
        self.robots().robot(1).posW(
            sva.PTransformd(sva.RotZ(0), eigen.Vector3d(0.0, 0.0, 0))
        )
        self.robots().robot(2).posW(
            sva.PTransformd(sva.RotZ(math.pi), eigen.Vector3d(2.0, 1.0, 0))
        )
        self.addContact(mc_control.Contact("dingo", "ur5e", "Base", "Base"))
        self._doorPosture = mc_tasks.PostureTask(self.qpsolver, 2, 1.0, 1.0)
        self.qpsolver.addTask(self._doorPosture)
        self._handTask = mc_tasks.SurfaceTransformTask(
            "Wrist", self.robots(), 0, 5.0, 1000.0
        )
        self._dingoEndEffectorTask = mc_tasks.EndEffectorTask(
            "base_link", self.robots(), 1, 1.0, 1000.0
        )
        self.qpsolver.addTask(self._dingoEndEffectorTask)
        self.qpsolver.addTask(self.postureTask)
        self.postureTask.target(
            {
                "shoulder_lift_joint".encode("utf-8"): [-math.pi / 2],
            }
        )
        self._dingoEndEffectorTask.add_ef_pose(
            sva.PTransformd(sva.RotZ(0), eigen.Vector3d(1.5, 0.0, 0.0))
        )
        self.count = 0

    @staticmethod
    def create(robot, dt):
        dingo = mc_rbdyn.get_robot_module(
            "object", "/usr/local/share/mc_dingo", "dingo"
        )
        door = mc_rbdyn.get_robot_module(
            "env", "/usr/share/mc_rtc/mc_int_obj_description", "door"
        )
        ground = mc_rbdyn.get_robot_module(
            "env", "/usr/share/mc_rtc/mc_env_description/", "ground"
        )
        return MobileArmController([robot, dingo, door, ground], dt)
