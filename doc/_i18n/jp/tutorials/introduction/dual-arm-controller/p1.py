# コンストラクタ内
def __init__(self, rm, dt):
    # ...
    ur_joints = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    self._ur_joints = [joint.encode("utf-8") for joint in ur_joints]
# resetコールバック関数内
def reset_callback(self, data):
    # ...
    self._urEndEffectorTask.selectUnactiveJoints(
        self.qpsolver, self._ur_joints
    )
    # ...
# runコールバック関数内
def run_callback(self):
    if self._phase == ControllerPhase.IDLE:
        self.postureTask.target(
            {
                "elbow_joint".encode("utf-8"): [-math.pi / 2],
                "wrist_2_joint".encode("utf-8"): [math.pi / 2],
            }
        )
        self._phase = ControllerPhase.STARTED
    # ...