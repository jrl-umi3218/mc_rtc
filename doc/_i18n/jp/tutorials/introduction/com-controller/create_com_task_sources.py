# mc_tasksモジュールをインポートする
import mc_tasks
# コンストラクター内でタスクを作成して問題に追加する
self.comTask = mc_tasks.CoMTask(self.robots(), 0, 10.0, 1000.0)
self.qpsolver.addTask(self.comTask)
# 姿勢制御タスクの剛性を下げる
self.postureTask.stiffness(1)
# resetコールバック関数内で、現在の質量中心に合わせてタスクをリセットする
self.comTask.reset()
