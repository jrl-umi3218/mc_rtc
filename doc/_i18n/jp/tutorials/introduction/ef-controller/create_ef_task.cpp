// エンドエフェクタータスクのヘッダーをインクルードする（ヘッダファイル） 
#include <mc_tasks/EndEffectorTask.h>
// クラスのプライベートメンバーを追加 (ヘッダファイル)
std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
// コンストラクター内でタスクを作成して問題に追加する
efTask = std::make_shared<mc_tasks::EndEffectorTask>("l_wrist", robots(), 0, 5.0, 500.0);
solver().addTask(efTask);
// reset関数内で、現在のエンドエフェクターの位置に合わせてタスクをリセットする
efTask->reset();
