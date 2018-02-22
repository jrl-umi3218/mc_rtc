#include <mc_tasks/MetaTask.h>

#include <mc_rtc/logging.h>

namespace mc_tasks
{

double extraStiffness(double error, double extraStiffness)
{
  return exp(-error)*extraStiffness;
}

MetaTask::~MetaTask() {}

void MetaTask::load(mc_solver::QPSolver & solver,
                    const mc_rtc::Configuration & config)
{
  if(config.has("dimWeight"))
  {
    Eigen::VectorXd dimW = config("dimWeight");
    if(dimW.size() != dimWeight().size())
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "Stored dimWeight has the wrong dimension (is " << dimW.size() << " should be " << dimWeight().size())
    }
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
  //gui.addElement(
  //  mc_rtc::gui::Element<void>{
  //    {"Tasks", name_, "Reset"},
  //    [this]() { this->reset(); }
  //  },
  //  mc_rtc::gui::Button{}
  //);
  //gui.addElement(
  //  mc_rtc::gui::Element<std::string>{
  //    {"Tasks", name_, "Details", "type"},
  //    std::function<std::string()>{[this]() { return this->type_; }}
  //  }
  //);
  //gui.addElement(
  //  mc_rtc::gui::Element<Eigen::VectorXd>{
  //    {"Tasks", name_, "Details", "eval"},
  //    std::function<Eigen::VectorXd()>{[this]() { return this->eval(); }}
  //  },
  //  mc_rtc::gui::Label(true)
  //);
  //gui.addElement(
  //  mc_rtc::gui::Element<Eigen::VectorXd>{
  //    {"Tasks", name_, "Details", "speed"},
  //    std::function<Eigen::VectorXd()>{[this]() { return this->speed(); }}
  //  },
  //  mc_rtc::gui::Label(true)
  //);
}

void MetaTask::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({"Tasks", name_});
}

std::function<bool(const mc_tasks::MetaTask & task, std::string&)>
  MetaTask::buildCompletionCriteria(double, const mc_rtc::Configuration &) const
{
  return [](const mc_tasks::MetaTask&, std::string&) { return true; };
}

}
