# 定数を宣言する
class ControllerState:
    GO = 0
    RETURN = 1
# コンストラクタ内
def __init__(self, rm, dt):
    self._ur_state = ControllerState.RETURN
    self._kinova_state = ControllerState.RETURN
# 2つのサポート関数を追加
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
# runコールバック関数内
def run_callback(self):
    # ...
    elif self._phase == ControllerPhase.MOVE:
        self._run_ur5e()
        self._run_kinova()
    return True