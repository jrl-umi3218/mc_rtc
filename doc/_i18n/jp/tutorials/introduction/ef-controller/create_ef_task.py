# mc_tasksモジュールをインポートする
import mc_tasks
# コンストラクター内でタスクを作成して問題に追加する
self.efTask = mc_tasks.EndEffectorTask("l_wrist", self.robots(), 0, 10.0, 1000.0)
self.qpsolver.addTask(self.efTask)
# resetコールバック関数内で、現在のエンドエフェクターの位置に合わせてタスクをリセットする
self.efTask.reset()
