#include <mc_observers/ObserverLoader.h>
#include <mc_observers/ObserverPipeline.h>
#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/io_utils.h>

namespace mc_observers
{

ObserverPipeline::ObserverPipeline(mc_control::MCController & ctl, const std::string & name) : ctl_(ctl), name_(name) {}

void ObserverPipeline::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void ObserverPipeline::reconfigure() {}

void ObserverPipeline::create(const mc_rtc::Configuration & config)
{
  std::vector<mc_rtc::Configuration> observersConf = config;
  observers_.reserve(observersConf.size());
  for(const auto & observerConf : observersConf)
  {
    if(!observerConf.has("type"))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[ObserverPipeline::{}] Observer {} requires a \"type\" configuration entry");
    }
    const std::string & observerType = observerConf("type");
    auto observerName = observerConf("name", observerType);
    auto update = observerConf("update", false);

    if(mc_observers::ObserverLoader::has_observer(observerType))
    {
      if(observersByName_.count(observerName))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[ObserverPipeline::{}] An observer named {} already exists (type: {}). Please make sure that observer "
            "names within a pipeline are unique, use the \"name\" configuration entry.",
            name_, observerName, observersByName_[observerName]->type());
      }
      auto observer = mc_observers::ObserverLoader::get_observer(observerType, observerName,
                                                                 observerConf("config", mc_rtc::Configuration{}));
      observer->name(observerName);
      observers_.push_back(observer);
      observersByName_[observerName] = observer;
      pipelineObservers_.push_back(std::make_pair(observer, update));
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[ObserverPipeline::{}] requires observer \"{}\" but it is not available.\n"
          "- Available observers are: {}\n"
          "- Note common reasons for this error include:\n"
          "\t- The library is not in a path read by mc_rtc\n"
          "\t- The library hasn't been properly linked\n",
          "\t- The observer name does not match the name exported by EXPORT_OBSERVER_MODULE\n",
          "\t- The constuctor segfaults\n", name_, observerName,
          mc_rtc::io::to_string(mc_observers::ObserverLoader::available_observers()));
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
    const auto & observerPair = pipelineObservers_[i];
    auto observer = observerPair.first;
    bool updateRobots = observerPair.second;
    observer->reset(ctl_);

    if(updateRobots)
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
  for(const auto & observerPair : pipelineObservers_)
  {
    auto observer = observerPair.first;
    bool updateRobots = observerPair.second;
    bool r = observer->run(ctl_);
    if(!r)
    {
      mc_rtc::log::error("Observer {} failed to run", observer->name());
      return false;
    }
    if(updateRobots)
    {
      observer->updateRobots(ctl_);
    }
  }
  return true;
}

void ObserverPipeline::addToLogger(mc_rtc::Logger & logger)
{
  for(auto & observer : observers_)
  {
    observer->addToLogger(logger, "Observers_" + name_);
  }
}
/*! \brief Remove observer from logger. */
void ObserverPipeline::removeFromLogger(mc_rtc::Logger & logger)
{
  for(auto & observer : observers_)
  {
    observer->removeFromLogger(logger, "Observers_" + name_);
  }
}

void ObserverPipeline::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  for(auto & observer : observers_)
  {
    observer->addToGUI(gui, {"ObserverPipeline", name_});
  }
}
void ObserverPipeline::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({"ObserverPipeline", name_});
}

} // namespace mc_observers
