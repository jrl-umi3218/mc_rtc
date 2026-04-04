/*
 * Copyright 2015-2024 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/gui/StateBuilder.h>

namespace mc_rbdyn::gui::lipm_stabilizer
{

/**
 * @brief Helper function to add LIMP
 *
 * @param gui State builder where the object is added (typically controller.gui())
 * @param category Category where the object is added (Main Category will be added to the given category)
 * @param config LIPM Stabilizer configuration
 */
MC_RBDYN_DLLAPI void addStabilizerToGUI(mc_rtc::gui::StateBuilder & gui,
                                        const std::vector<std::string> & category,
                                        mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & config);

} // namespace mc_rbdyn::gui::lipm_stabilizer
