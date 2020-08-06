#include <mc_observers/ObserverLoader.h>
#include <mc_observers/ObserverPipeline.h>
#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/io_utils.h>

namespace mc_observers
{

ObserverPipeline::ObserverPipeline(mc_control::MCController & ctl, const std::string & name) : ctl_(ctl), name_(name) {}
ObserverPipeline::ObserverPipeline(mc_control::MCController & ctl) : ctl_(ctl) {}

void ObserverPipeline::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void ObserverPipeline::reconfigure() {}

void ObserverPipeline::create(const mc_rtc::Configuration & config, double dt)
{
  if(!config.has("name"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[ObserverPipeline] \"name\" entry is required", name_);
  }
  name_ = static_cast<std::string>(config("name"));
  auto observersConfs = mc_rtc::fromVectorOrElement(config, "observers", std::vector<mc_rtc::Configuration>{});
  observers_.reserve(observersConfs.size());
  for(const auto & observerConf : observersConfs)
  {
    if(!observerConf.has("type"))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[ObserverPipeline::{}] Observer {} requires a \"type\" configuration entry");
    }
    const std::string & observerType = observerConf("type");
    auto observerName = observerConf("name", observerType);

    if(mc_observers::ObserverLoader::has_observer(observerType))
    {
      if(observersByName_.count(observerName))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[ObserverPipeline::{}] An observer named {} already exists (type: {}). Please make sure that observer "
            "names within a pipeline are unique, use the \"name\" configuration entry.",
            name_, observerName, observersByName_[observerName]->type());
      }
      auto observer = mc_observers::ObserverLoader::get_observer(observerType, dt);
      observer->name(observerName);
      observer->configure(ctl_, observerConf("config", mc_rtc::Configuration{}));
      observers_.push_back(observer);
      observersByName_[observerName] = observer;
      pipelineObservers_.emplace_back(observer, observerConf("update", true), observerConf("log", true),
                                      observerConf("gui", true));
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
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
  if(pipelineObservers_.empty())
  {
    desc_ += "None";
  }

  for(size_t i = 0; i < pipelineObservers_.size(); ++i)
  {
    const auto & pipelineObserver = pipelineObservers_[i];
    auto & observer = pipelineObserver.observer;
    observer->reset(ctl_);

    if(pipelineObserver.update)
    {
      observer->updateRobots(ctl_);
      desc_ += observer->desc();
    }
    else
    {
      desc_ += "[" + observer->desc() + "]";
    }

    if(i < pipelineObservers_.size() - 1)
    {
      desc_ += " -> ";
    }
  }
}

bool ObserverPipeline::run()
{
  success_ = true;
  for(auto & pipelineObserver : pipelineObservers_)
  {
    auto & observer = pipelineObserver.observer;
    bool res = observer->run(ctl_);
    success_ = success_ && res;
    if(!res)
    {
      if(pipelineObserver.previousSuccess)
      {
        mc_rtc::log::warning("[ObserverPipeline::{}] Observer {} failed to run", name(), observer->name());
        if(observer->error().size())
        {
          mc_rtc::log::warning("{}", observer->error());
        }
        pipelineObserver.previousSuccess = false;
      }
    }
    else
    {
      if(!pipelineObserver.previousSuccess)
      {
        mc_rtc::log::info("[ObserverPipeline::{}] Observer {} resumed", name(), observer->name());
        pipelineObserver.previousSuccess = true;
      }
      if(pipelineObserver.update)
      {
        observer->updateRobots(ctl_);
      }
    }
  }
  return success_;
}

void ObserverPipeline::addToLogger()
{
  for(auto & observer : pipelineObservers_)
  {
    if(observer.log)
    {
      observer.observer->addToLogger_(ctl_, "Observers_" + name_);
    }
  }
}
/*! \brief Remove observer from logger. */
void ObserverPipeline::removeFromLogger()
{
  for(auto & observer : pipelineObservers_)
  {
    if(observer.log)
    {
      observer.observer->removeFromLogger_(ctl_, "Observers_" + name_);
    }
  }
}

void ObserverPipeline::addToGUI()
{
  for(auto & observer : pipelineObservers_)
  {
    if(observer.gui)
    {
      observer.observer->addToGUI_(ctl_, {"ObserverPipelines", name_});
    }
  }
}
void ObserverPipeline::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({"ObserverPipelines", name_});
}

} // namespace mc_observers
