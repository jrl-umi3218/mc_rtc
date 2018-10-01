#include <mc_rtc/logging.h>
#include <mc_tasks/MetaTask.h>

namespace mc_tasks
{

double extraStiffness(double error, double extraStiffness)
{
  return exp(-error) * extraStiffness;
}

MetaTask::~MetaTask() {}

void MetaTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("dimWeight"))
  {
    Eigen::VectorXd dimW = config("dimWeight");
    if(dimW.size() != dimWeight().size())
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "Stored dimWeight has the wrong dimension (is "
                                                  << dimW.size() << " should be " << dimWeight().size() << ")")
    }
    dimWeight(dimW);
  }
  if(config.has("activeJoints"))
  {
    selectActiveJoints(solver, config("activeJoints"));
  }
  if(config.has("unactiveJoints"))
  {
    selectUnactiveJoints(solver, config("unactiveJoints"));
  }
  if(config.has("name"))
  {
    name(config("name"));
  }
}

void MetaTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({"Tasks", name_}, mc_rtc::gui::Button("Reset", [this]() { this->reset(); }));
  gui.addElement({"Tasks", name_, "Details"}, mc_rtc::gui::ArrayLabel("eval", [this]() { return this->eval(); }),
                 mc_rtc::gui::ArrayLabel("speed", [this]() { return this->speed(); }),
                 mc_rtc::gui::Label("type", [this]() { return this->type_; }));
  if(dimWeight().size())
  {
    gui.addElement({"Tasks", name_, "Gains", "Dimensional"},
                   mc_rtc::gui::ArrayInput("weight", [this]() { return this->dimWeight(); },
                                           [this](const Eigen::VectorXd & v) { this->dimWeight(v); }));
  }
}

void MetaTask::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({"Tasks", name_});
}

std::function<bool(const mc_tasks::MetaTask & task, std::string &)> MetaTask::buildCompletionCriteria(
    double,
    const mc_rtc::Configuration &) const
{
  return [](const mc_tasks::MetaTask &, std::string &) { return true; };
}

} // namespace mc_tasks
