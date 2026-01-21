# 定数を宣言する
class ControllerPhase:
    IDLE = 0
    STARTED = 1
    MOVE = 2


# コンストラクター内
def __init__(self, rm, dt):
    self._phase = IDLE


# runコールバック関数内
def run_callback(self):
    if self._phase == ControllerPhase.IDLE and False:  # この条件は後で記述する
        # STARTEDフェーズをセットアップする
        self._phase = ControllerPhase.STARTED
    elif self._phase == ControllerPhase.STARTED and False:  # この条件は後で記述する
        # MOVEフェーズをセットアップする
        self._phase = ControllerPhase.MOVE
    elif self._phase == ControllerPhase.MOVE and False:  # この条件は後で記述する
        # MOVEフェーズを継続する
        pass
    return True
