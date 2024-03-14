/*
 * Copyright 2015-2024 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robot.h>
#include <mc_rtc/gui/StateBuilder.h>

namespace mc_rbdyn::gui
{

static const mc_rtc::gui::LineConfig defaultSurfaceConfig = []()
{ return mc_rtc::gui::LineConfig{mc_rtc::gui::Color::Green, 0.01}; }();

/** Helper function to create a GUI element from a surface object inside a robot
 *
 * Multiple elements can be added based on the surface type (e.g. for a planar surface, the polygon and the normal are
 * added) so this function returns the names of the elements that were added
 *
 * \param gui State builder where the object is added (typically controller.gui()
 *
 * \param category Category where the object is added
 *
 * \param cfg Configuration for the surface polygon
 *
 * \param robot Robot that the surfaces belongs to, this reference is captured by the GUI and should survive
 *
 * \param name Name of the surface added to the GUI, the object should remain in the robot while it is in the
 * GUI
 *
 * \param publishName Name of the object in the GUI, defaults to \param name
 *
 * \returns The names of the elements added by the function
 */
MC_RBDYN_DLLAPI std::vector<std::string> addSurfaceToGUI(mc_rtc::gui::StateBuilder & gui,
                                                         const std::vector<std::string> & category,
                                                         const mc_rbdyn::Robot & robot,
                                                         const std::string & name,
                                                         const mc_rtc::gui::LineConfig & cfg = defaultSurfaceConfig,
                                                         const std::optional<std::string> & publishName = std::nullopt);

} // namespace mc_rbdyn::gui
