/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/*! Static interface used to load controllers, this is only used in static builds of mc_rtc */

#include <mc_control/MCController.h>

#include <mc_rtc/config.h>
#include <mc_rtc/loader.h>

#include <map>
#include <memory>
#include <mutex>

namespace mc_control
{

/*! \class ControllerLoader
 *
 * A wrapper around a singleton mc_rtc::ObjectLoader<MCController>
 */
struct MC_CONTROL_DLLAPI ControllerLoader
{
public:
  static mc_rtc::ObjectLoader<MCController> & loader();

private:
  static std::unique_ptr<mc_rtc::ObjectLoader<MCController>> loader_;
};

} // namespace mc_control
