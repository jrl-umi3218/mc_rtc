/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/*! Static interface used to load plugins, this is only used in static builds of mc_rtc */

#include <mc_control/GlobalPlugin.h>

#include <mc_rtc/config.h>
#include <mc_rtc/loader.h>

#include <map>
#include <memory>
#include <mutex>

namespace mc_control
{

/*! \class GlobalPluginLoader
 *
 * A wrapper around a singleton mc_rtc::ObjectLoader<GlobalPlugin>
 */
struct MC_CONTROL_DLLAPI GlobalPluginLoader
{
public:
  static mc_rtc::ObjectLoader<GlobalPlugin> & loader();

private:
  static std::unique_ptr<mc_rtc::ObjectLoader<GlobalPlugin>> loader_;
};

} // namespace mc_control
