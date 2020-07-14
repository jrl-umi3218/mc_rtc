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
  const auto & cc = config;
  auto runObservers = mc_rtc::fromVectorOrElement<std::string>(cc, "RunObservers", {});
  auto updateObservers = mc_rtc::fromVectorOrElement<std::string>(cc, "UpdateObservers", {});
  std::vector<ObserverPtr> loadedObservers;
  loadedObservers.reserve(runObservers.size());
  // Load observers
  for(const auto & observerName : runObservers)
  {
    if(mc_observers::ObserverLoader::has_observer(observerName))
    {
      auto observer = mc_observers::ObserverLoader::get_observer(observerName, config);
      loadedObservers.push_back(observer);
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Observer pipeline \"{}\" requires observer \"{}\" but it is not available.\n"
          "- Available observers are: {}\n"
          "- Note common reasons for this error include:\n"
          "\t- The library is not in a path read by mc_rtc\n"
          "\t- The library hasn't been properly linked",
          "\t- The observer name does not match the name exported by EXPORT_OBSERVER_MODULE\n",
          "\t- The constuctor segfaults\n", observerName,
          mc_rtc::io::to_string(mc_observers::ObserverLoader::available_observers()));
    }
  }
  observers(loadedObservers);

  // Configure the pipeline
  for(const auto & observerName : runObservers)
  {
    if(observersByName_.count(observerName) > 0)
    {
      auto observer = observersByName_[observerName];
      // If observer is in the "UpdateObserver" configuration, request for
      // update
      if(std::find(updateObservers.begin(), updateObservers.end(), observerName) != updateObservers.end())
      {
        pipelineObservers_.push_back(std::make_pair(observer, true));
      }
      else
      {
        pipelineObservers_.push_back(std::make_pair(observer, false));
      }
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[ObserverPipeline::{}] Requested observer {} but this observer is not available. Please make sure that it "
          "is in "
          "your \"EnabledObservers\" configuration, and that is was properly loaded.",
          name_, observerName);
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
    observer->addToLogger(logger, guiCategory_ + "_" + name_);
  }
}
/*! \brief Remove observer from logger. */
void ObserverPipeline::removeFromLogger(mc_rtc::Logger & logger)
{
  for(auto & observer : observers_)
  {
    observer->removeFromLogger(logger, guiCategory_ + "_" + name_);
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
