/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
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

void MessageState::start(Controller & ctl)
{
  if(type_ == "info")
  {
    LOG_INFO(message_);
  }
  if(type_ == "success")
  {
    LOG_SUCCESS(message_);
  }
  if(type_ == "warning")
  {
    LOG_WARNING(message_);
  }
  if(type_ == "error")
  {
    LOG_ERROR(message_);
  }
}

bool MessageState::run(Controller &)
{
  output("OK");
  return true;
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("Message", mc_control::fsm::MessageState)
