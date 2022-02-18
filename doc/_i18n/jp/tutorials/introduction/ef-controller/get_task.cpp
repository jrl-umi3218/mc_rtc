// タスクローダーを取得する
#include <mc_tasks/MetaTaskLoader.h>

// JSONファイルからタスクを取得する
auto task = mc_tasks::MetaTaskLoader::load(solver(), "/my/path/task.json");

// YAMLファイルからタスクを取得する
auto task = mc_tasks::MetaTaskLoader::load(solver(), "/my/path/task.yaml");

// 実際には、任意のmc_rtc::Configurationエントリからタスクを取得できる
auto task = mc_tasks::MetaTaskLoader::load(solver(), config("task"));

// 上記のいずれの場合も、タスクはstd::shared_ptrとして取得される。<mc_tasks::MetaTask>
// ただし、型を具体的に指定して取得することもできる。
// ディスクから取得されたタスクとユーザーが要求したタスクとの間に
// 互換性があるかどうかがmc_rtcによってチェックされる
auto task = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(solver(), config("task"));
