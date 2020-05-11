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
  config("log", log_);
  config("log_type", type_);
  config("gui", gui_);
  config("gui_category", category_);
}

void MessageState::start(Controller & ctl)
{
  std::transform(type_.begin(), type_.end(), type_.begin(), [](unsigned char c) { return std::tolower(c); });

  if(log_)
  {
    std::string message;
    if(prefix_.size())
    {
      message += "[" + name() + "::" + prefix_ + "] ";
    }
    else
    {
      message += "[" + name() + "]";
    }

    if(type_ == "info")
    {
      LOG_INFO(message);
    }
    else if(type_ == "success")
    {
      LOG_SUCCESS(message);
    }
    else if(type_ == "warning")
    {
      LOG_WARNING(message);
    }
    else if(type_ == "error")
    {
      LOG_ERROR(message);
    }
  }

  if(gui_)
  {
    labelName_ = prefix_.size() ? prefix_ : name();
    ctl.gui()->addElement(category_,
                          mc_rtc::gui::Label(labelName_, [this]() -> const std::string & { return message_; }));
  }
  else
  {
    LOG_WARNING("[" << name() << "] Unknown type: " << type_ << ", treating as INFO")
    LOG_INFO(message_);
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
    ctl.gui()->removeElement(category_, labelName_);
  }
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("Message", mc_control::fsm::MessageState)
