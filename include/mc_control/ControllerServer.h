/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/ControllerServerConfiguration.h>
#include <mc_control/MCController.h>

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>

#include <string>
#include <vector>

namespace mc_control
{

/** Serves data and allow interaction with the controllers
 *
 * - Uses a PUB socket to send the data stream
 *
 * - Uses a PULL socket to handle requests
 */
struct MC_CONTROL_DLLAPI ControllerServer
{

  /** Constructor
   *
   * \param dt Controller timestep
   *
   * \param server_dt Publication timestep
   *
   * \param pub_bind_uri List of URI the PUB socket should bind to
   *
   * \param pull_bind_uri List of URI the PULL socket should bind to
   *
   * Check nanomsg documentation for supported protocols
   */
  ControllerServer(double dt,
                   double server_dt,
                   const std::vector<std::string> & pub_bind_uri,
                   const std::vector<std::string> & pull_bind_uri);

  /** Construct from the provided configuration
   *
   * \param dt Controller timestep
   *
   * \param config Server configuration \ref ControllerServerConfiguration for details
   */
  ControllerServer(double dt, const ControllerServerConfiguration & config);

  ~ControllerServer();

  /** Handle requests made by the GUI users */
  void handle_requests(mc_rtc::gui::StateBuilder & gui_builder);

  /** Handle requests from raw data */
  void handle_requests(mc_rtc::gui::StateBuilder & gui, const char * data);

  /** Publish the current GUI state */
  void publish(mc_rtc::gui::StateBuilder & gui_builder);

  /** Get latest published data */
  std::pair<const char *, size_t> data() const;

  /** Attach a logger to the server */
  inline void set_logger(std::shared_ptr<mc_rtc::Logger> logger) noexcept { logger_ = logger; }

  /** Set requests to handle on the next iteration */
  inline void push_requests(const std::vector<mc_rtc::Logger::GUIEvent> & requests)
  {
    for(const auto & r : requests) { requests_.push_back(r); }
  }

  /** Update the rate of the server */
  void update_rate(double dt, double server_dt);

private:
  unsigned int iter_;
  unsigned int rate_;

  int pub_socket_;
  int pull_socket_;

  std::vector<char> buffer_;
  size_t buffer_size_ = 0;

  std::shared_ptr<mc_rtc::Logger> logger_;

  std::vector<mc_rtc::Logger::GUIEvent> requests_;
};

} // namespace mc_control
