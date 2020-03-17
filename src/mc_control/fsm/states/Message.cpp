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
  config("message", message_);
  config("type", type_);
}

void MessageState::start(Controller & /* ctl */)
{
  std::transform(type_.begin(), type_.end(), type_.begin(), [](unsigned char c) { return std::tolower(c); });

  if(type_ == "info")
  {
    LOG_INFO(message_);
  }
  else if(type_ == "success")
  {
    LOG_SUCCESS(message_);
  }
  else if(type_ == "warning")
  {
    LOG_WARNING(message_);
  }
  else if(type_ == "error")
  {
    LOG_ERROR(message_);
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

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("Message", mc_control::fsm::MessageState)
