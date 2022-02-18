# 定数を宣言する
APPROACH = 0
HANDLE = 1
OPEN = 2
# コンストラクター内
self.phase = APPROACH
# コントローラーの新しいメソッド
def switch_phase(self):
  if self.phase == APPROACH and False: # この条件は後で記述する
    # HANDLEフェーズをセットアップする
    self.phase = HANDLE
  elif self.phase == HANDLE and False: # この条件は後で記述する
    # OPENフェーズをセットアップする
    self.phase = OPEN
# runコールバック関数内でこれを呼び出す
def run_callback(self):
    self.switch_phase()
    return True
