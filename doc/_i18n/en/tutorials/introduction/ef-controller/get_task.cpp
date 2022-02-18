// Get the task loader in there
#include <mc_tasks/MetaTaskLoader.h>

// Get the task from the JSON file
auto task = mc_tasks::MetaTaskLoader::load(solver(), "/my/path/task.json");

// Get the task from the YAML file
auto task = mc_tasks::MetaTaskLoader::load(solver(), "/my/path/task.yaml");

// Actually you can get the task from any mc_rtc::Configuration entries
auto task = mc_tasks::MetaTaskLoader::load(solver(), config("task"));

// In all the cases above, task is an std::shared_ptr<mc_tasks::MetaTask>
// but you can retrieve a more precise type, mc_rtc will check that the
// task that was retrieved from disk is compatible with the task you
// requested
auto task = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(solver(), config("task"));
