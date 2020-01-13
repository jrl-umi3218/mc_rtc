/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_control/mc_global_controller.h>

namespace mc_control
{

/** \class Global plugin base-class
 *
 * This interface must be implemented by each global plugin.
 *
 * A global plugin is loaded by an instance of \ref
 * mc_control::MCGlobalController and performs operations before and/or after
 * \ref mc_control::MCGlobalController::run()
 *
 */
struct MC_CONTROL_DLLAPI GlobalPlugin
{
  virtual ~GlobalPlugin() = default;

  /** Initialize the plugin
   *
   * This function is called when the plugin is created by the
   * MCGlobalController instance
   *
   * The plugin configuration is read from:
   * - $PLUGIN_DIR/etc/$PLUGIN_NAME.yaml
   * - $HOME/.config/mc_rtc/plugins/$PLUGIN_NAME.yaml on Linux/macOS
   * - %APPDATA%/mc_rtc/plugins/$PLUGIN_NAME.yaml on Windows
   *
   * \param controller MCGlobalController instance that owns this plugin
   *
   * \param config Plugin configuration
   *
   */
  virtual void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) = 0;

  /** Reset the plugin
   *
   * This function is called when the controller is switched
   *
   * \param controller MCGlobalController instance that owns this plugin
   *
   */
  virtual void reset(mc_control::MCGlobalController & controller) = 0;

  /** Run before \ref mc_control::MCGlobalController::run()
   *
   * \param controller MCGlobalController instance that owns this plugin
   *
   */
  virtual void before(mc_control::MCGlobalController & controller) = 0;

  /** Run after \ref mc_control::MCGlobalController::run()
   *
   * \param controller MCGlobalController instance that owns this plugin
   *
   */
  virtual void after(mc_control::MCGlobalController & controller) = 0;
};

} // namespace mc_control

#ifdef WIN32
#  define GLOBAL_PLUGIN_API __declspec(dllexport)
#else
#  if __GNUC__ >= 4
#    define GLOBAL_PLUGIN_API __attribute__((visibility("default")))
#  else
#    define GLOBAL_PLUGIN_API
#  endif
#endif
