/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/Message.h>

namespace mc_control
{

namespace fsm
{

void MessageState::configure(const mc_rtc::Configuration & config)
{
  config("prefix", prefix_);
  config("message", message_);
  config("log", logType_);
  if(config.has("gui"))
  {
    gui_ = true;
    guiCategory_ = config("gui");
  }
}

void MessageState::start(Controller & ctl)
{
  std::transform(logType_.begin(), logType_.end(), logType_.begin(), [](unsigned char c) { return std::tolower(c); });

  if(logType_.size())
  {
    std::string prefix;
    std::string message;
    if(prefix_.size())
    {
      prefix = "[" + name() + "::" + prefix_ + "] ";
    }
    else
    {
      prefix = "[" + name() + "] ";
    }
    message = prefix + message_;

    if(logType_ == "info")
    {
      LOG_INFO(message);
    }
    else if(logType_ == "success")
    {
      LOG_SUCCESS(message);
    }
    else if(logType_ == "warning")
    {
      LOG_WARNING(message);
    }
    else if(logType_ == "error")
    {
      LOG_ERROR(message);
    }
    else if(logType_ == "none")
    { /* Do not log anything to the terminal */
    }
    else
    {
      LOG_ERROR(prefix << " Provided log type " << logType_
                       << " is invalid, assuming info. Supported types are [info, success, warning, error, none]");
      LOG_INFO(message);
    }
  }

  if(gui_)
  {
    labelName_ = prefix_.size() ? prefix_ : name();
    ctl.gui()->addElement(guiCategory_,
                          mc_rtc::gui::Label(labelName_, [this]() -> const std::string & { return message_; }));
  }
  output("OK");
}

bool MessageState::run(Controller &)
{
  return true;
}

void MessageState::teardown(Controller & ctl)
{
  if(gui_)
  {
    ctl.gui()->removeElement(guiCategory_, labelName_);
  }
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("Message", mc_control::fsm::MessageState)
