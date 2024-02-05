/*
 * Copyright 2015-2024 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/gui/types.h>

namespace mc_rbdyn::gui
{

static const mc_rtc::gui::PolyhedronConfig defaultConvexConfig = []()
{
  mc_rtc::gui::PolyhedronConfig cfg;
  cfg.triangle_color = {0, 0.8, 0, 0.5};
  cfg.edge_config.color = {0, 1, 0, 1};
  cfg.edge_config.width = 0.001;
  cfg.show_edges = true;
  cfg.show_vertices = false;
  return cfg;
}();

/** Helper function to create a GUI element from a convex object inside a robot
 *
 * \param gui State builder where the object is added (typically controller.gui()
 *
 * \param category Category where the object is added
 *
 * \param cfg Configuration of the appearance of the convex object
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
                                    const mc_rtc::gui::PolyhedronConfig & cfg = defaultConvexConfig,
                                    const std::optional<std::string> & publishName = std::nullopt);

} // namespace mc_rbdyn::gui
