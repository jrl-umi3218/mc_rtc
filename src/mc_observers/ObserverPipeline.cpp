#include <mc_observers/ObserverPipeline.h>

#include <mc_observers/ObserverLoader.h>

#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/path.h>

#include <mc_control/MCController.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_observers
{

static inline std::string get_config(const std::string & dir, const std::string & name)
{
  bfs::path cfg = bfs::path(dir) / name;
  for(const auto & ext : {".conf", ".yaml", ".yml"})
  {
    cfg.replace_extension(ext);
    if(bfs::exists(cfg)) { return cfg.string(); }
  }
  return "";
}

static inline void load_config(mc_rtc::Configuration & out, const std::string & dir, const std::string & name)
{
  auto cfg = get_config(dir, name);
  if(cfg.empty()) { return; }
  mc_rtc::log::info("Loading additional observer configuration from {}", cfg);
  out.load(mc_rtc::Configuration(cfg));
}

static inline mc_rtc::Configuration get_observer_config(const std::string & observerType,
                                                        const std::string & robot,
                                                        mc_rtc::Configuration config)
{
  mc_rtc::Configuration out;
  // Load observer configuration
  auto runtime_dir = mc_observers::ObserverLoader::get_observer_runtime_directory(observerType);
  if(!runtime_dir.empty()) { load_config(out, runtime_dir + "/etc", observerType); }
  bfs::path user_path = mc_rtc::user_config_directory_path("observers");
  load_config(out, user_path.string(), observerType);
  // Load robot specific configuration
  if(!runtime_dir.empty()) { load_config(out, runtime_dir + "/" + observerType, robot); }
  load_config(out, (user_path / observerType).string(), robot);
  // Finally load the configuration provided in the pipeline
  out.load(config);
  return out;
}

ObserverPipeline::ObserverPipeline(mc_control::MCController & ctl, const std::string & name) : ctl_(ctl), name_(name) {}
ObserverPipeline::ObserverPipeline(mc_control::MCController & ctl) : ctl_(ctl) {}

void ObserverPipeline::create(const mc_rtc::Configuration & config, double dt)
{
  if(!config.has("name")) { mc_rtc::log::error_and_throw("[ObserverPipeline] \"name\" entry is required", name_); }
  name_ = static_cast<std::string>(config("name"));
  config("run", runObservers_);
  config("update", updateObservers_);
  auto observersConfs = mc_rtc::fromVectorOrElement(config, "observers", std::vector<mc_rtc::Configuration>{});
  for(const auto & observerConf : observersConfs)
  {
    if(!observerConf.has("type"))
    {
      mc_rtc::log::error_and_throw("[ObserverPipeline::{}] Observer {} requires a \"type\" configuration entry");
    }
    const std::string & observerType = observerConf("type");
    auto observerName = observerConf("name", observerType);

    if(mc_observers::ObserverLoader::has_observer(observerType))
    {
      if(hasObserver(observerName))
      {
        mc_rtc::log::error_and_throw(
            "[ObserverPipeline::{}] An observer named {} already exists (type: {}). Please make sure that observer "
            "names within a pipeline are unique, use the \"name\" configuration entry.",
            name_, observerName, observerType);
      }
      auto observer = mc_observers::ObserverLoader::get_observer(observerType, dt);
      observer->name(observerName);
      auto config = observerConf("config", mc_rtc::Configuration{});
      std::string robot = config("robot", ctl_.robot().name());
      if(ctl_.hasRobot(robot)) { robot = ctl_.robot(robot).module().name; }
      observer->configure(ctl_, get_observer_config(observerType, robot, config));
      pipelineObservers_.emplace_back(observer, observerConf);
    }
    else if(!observerConf("required", true))
    {
      mc_rtc::log::warning(
          "[ObserverPipeline::{}] Optional observer \"{}\" is in the observer pipeline configuration but "
          "is not available, ignoring it.",
          name_, observerName);
    }
    else
    {
      mc_rtc::log::error_and_throw(
          "[ObserverPipeline::{}] requires observer \"{}\" but it is not available (available: [{}])\n"
          "Common reasons for this error include:\n"
          "  - The observer library hasn't been properly linked\n"
          "  - The observer library is not in a path read by mc_rtc\n"
          "  - The observer type does not match the one exported by EXPORT_OBSERVER_MODULE\n"
          "  - The observer constuctor segfaults\n",
          name_, observerName, mc_rtc::io::to_string(mc_observers::ObserverLoader::available_observers()));
    }
  }
}

void ObserverPipeline::reset()
{
  desc_ = name_ + ": ";
  if(pipelineObservers_.empty()) { desc_ += "None"; }

  for(size_t i = 0; i < pipelineObservers_.size(); ++i)
  {
    auto & pipelineObserver = pipelineObservers_[i];
    auto & observer = pipelineObserver.observer();
    observer.reset(ctl_);

    if(pipelineObserver.update()) { desc_ += observer.desc(); }
    else { desc_ += "[" + observer.desc() + "]"; }

    if(i < pipelineObservers_.size() - 1) { desc_ += " -> "; }
  }
}

bool ObserverPipeline::run()
{
  if(!runObservers_) return true;
  success_ = true;
  for(auto & pipelineObserver : pipelineObservers_)
  {
    auto & observer = pipelineObserver.observer();
    bool res = observer.run(ctl_);
    if(!res)
    {
      if(pipelineObserver.successRequired()) { success_ = false; }
      if(pipelineObserver.success())
      {
        mc_rtc::log::warning("[ObserverPipeline::{}] Observer {} failed to run", name(), observer.name());
        if(observer.error().size()) { mc_rtc::log::warning("{}", observer.error()); }
        pipelineObserver.success_ = false;
      }
    }
    else
    {
      if(!pipelineObserver.success())
      {
        mc_rtc::log::info("[ObserverPipeline::{}] Observer {} resumed", name(), observer.name());
        pipelineObserver.success_ = true;
      }
      if(updateObservers_ && pipelineObserver.update_) { observer.update(ctl_); }
    }
  }
  return success_;
}

void ObserverPipeline::addToLogger(mc_rtc::Logger & logger)
{
  for(auto & observer : pipelineObservers_)
  {
    if(observer.log()) { observer.observer().addToLogger_(ctl_, logger, "Observers_" + name_); }
  }
}
/*! \brief Remove observer from logger. */
void ObserverPipeline::removeFromLogger(mc_rtc::Logger & logger)
{
  for(auto & observer : pipelineObservers_)
  {
    if(observer.log()) { observer.observer().removeFromLogger_(logger, "Observers_" + name_); }
  }
}

void ObserverPipeline::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  for(auto & observer : pipelineObservers_)
  {
    if(observer.gui()) { observer.observer().addToGUI_(ctl_, gui, {"ObserverPipelines", name_}); }
  }
}
void ObserverPipeline::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({"ObserverPipelines", name_});
}

} // namespace mc_observers
