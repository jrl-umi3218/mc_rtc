// 質量中心タスクのヘッダーをインクルードする（ヘッダー）
#include <mc_tasks/CoMTask.h>
// クラス内のプライベートメンバー（ヘッダー）
std::shared_ptr<mc_tasks::CoMTask> comTask;
// コンストラクター内でタスクを作成して問題に追加する
comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
solver().addTask(comTask);
// 姿勢制御タスクの剛性を下げる
postureTask->stiffness(1);
// reset関数内で、現在の質量中心に合わせてタスクをリセットする
comTask->reset();
