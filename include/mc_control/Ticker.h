/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/mc_global_controller.h>

#include <mc_rtc/log/FlatLog.h>

#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

namespace mc_control
{

/** A ticker runs a (potentially infinite) loop of MCGlobalController
 *
 * The default behavior is to run in open-loop mode where the outputs of mc_rtc are used to feed the inputs of
 * MCGlobalController
 *
 * It also supports loading log files to replay inputs, in replay we can:
 * - replay the sensor inputs
 * - replay the GUI inputs
 * - replay arbitrary data into the datastore
 * - replay outputs
 *
 */
struct MC_CONTROL_DLLAPI Ticker
{
  /** Configuration for the Ticker operation */
  struct Configuration
  {
    /** Configuration file for mc_rtc */
    std::string mc_rtc_configuration = "";
    /** If true start in step-by-step mode */
    bool step_by_step = false;
    /** Run for the specified time (seconds) */
    double run_for = std::numeric_limits<double>::infinity();
    /** Disable ticker time synchronization with real time */
    bool no_sync = false;
    /** Target ratio for sim/real */
    double sync_ratio = 1.0;
    /** Replay configuration */
    struct Replay
    {
      /** Replay log used by this instance */
      std::string log = "";
      /** If true, replay sensor inputs from the log to the controller */
      bool with_inputs = true;
      /** If true, replay GUI inputs from the log to the controller */
      bool with_gui_inputs = true;
      /** A file describing the mapping of log inputs to datastore inputs, it is expected to be a YAML file describing
       * the name to name correspondance */
      std::string with_datastore_config = "";
      /** If true, replay the outputs from the log to the controller */
      bool with_outputs = false;
      /** If true, stops when the log is finished, otherwise continue to run afterwards */
      bool stop_after_log = true;
      /** If true, exit when the log is finished, otherwise continue to run afterwards */
      bool exit_after_log = false;
    };
    Replay replay_configuration = {};
  };

  /** Initialize the ticker with the given configuration, this also initialize the controller */
  Ticker(const Configuration & config);

  /** Reset the ticker to the initial configuration
   *
   * Note: the reset happes on the next call to \ref step()
   */
  void reset();

  /** Run one step
   *
   * This function does not do any time synchronization
   */
  bool step();

  /** Run as many iterations as configured */
  void run();

  /** Elapsed simulation time since the last reset */
  inline double elapsed_time() const noexcept { return static_cast<double>(iters_) * gc_.timestep(); }

  /** Access the controller (const) */
  inline const mc_control::MCGlobalController & controller() const noexcept { return gc_; }

  /** Access the controller */
  inline mc_control::MCGlobalController & controller() noexcept { return gc_; }

protected:
  Configuration config_;
  mc_control::MCGlobalController gc_;
  std::shared_ptr<mc_rtc::log::FlatLog> log_;

  /** Number of steps taken since the last reset */
  size_t iters_ = 0;

  /** Current sim/real ratio */
  double sim_real_ratio_ = 1.0;

  /** Do a reset on the next iteration */
  std::atomic<bool> do_reset_ = false;

  /** True while running */
  bool running_ = true;

  /** True when the replay is done */
  bool replay_done_ = false;

  /** Number of steps remaning before going back to pause */
  int64_t rem_steps_ = 0;

  void simulate_sensors();

  void setup_gui();

  void set_time(double t);
};

} // namespace mc_control
