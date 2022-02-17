// ヘッダー内
#include <mc_tasks/SurfaceTransformTask.h>
// プライベートメンバー内
std::shared_ptr<mc_tasks::SurfaceTransformTask> handTask;
// reset関数内
// タスクを作成してソルバーに追加する
handTask = std::make_shared<mc_tasks::SurfaceTransformTask>("RightGripper", robots(), 0, 5.0, 1000.0);
solver().addTask(handTask);
// ハンドルの位置を基準とした目標を設定する
handTask->target(sva::PTransformd(Eigen::Vector3d(0, 0, -0.025)) * robots().robot(1).surfacePose("Handle"));
