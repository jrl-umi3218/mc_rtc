/*
 * Copyright 2015-2024 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robot.h>
#include <mc_rtc/gui/StateBuilder.h>

namespace mc_rbdyn::gui
{

/** Helper function to create a GUI element from a convex object inside a robot
 *
 * \param gui State builder where the object is added (typically controller.gui()
 *
 * \param category Category where the object is added
 *
 * \param robot Robot that the convex belongs to, this reference is captured by the GUI and should survive
 *
 * \param name Name of the collision object added to the GUI, the object should remain in the robot while it is in the
 * GUI
 *
 * \param publishName Name of the object in the GUI, defaults to \param name
 */
MC_RBDYN_DLLAPI void addConvexToGUI(mc_rtc::gui::StateBuilder & gui,
                                    const std::vector<std::string> & category,
                                    const mc_rbdyn::Robot & robot,
                                    const std::string & name,
                                    const std::optional<std::string> & publishName = std::nullopt);

} // namespace mc_rbdyn::gui
