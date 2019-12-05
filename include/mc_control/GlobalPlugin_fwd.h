/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_rtc/loader.h>

/* Forward declarations for GlobalPlugin */

namespace mc_control
{

struct GlobalPlugin;
using GlobalPluginPtr = std::unique_ptr<GlobalPlugin, mc_rtc::ObjectLoader<GlobalPlugin>::ObjectDeleter>;

} // namespace mc_control
