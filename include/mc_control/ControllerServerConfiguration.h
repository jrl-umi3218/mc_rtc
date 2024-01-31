/*
 * Copyright 2015-2024 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>

#include <mc_rtc/Configuration.h>
#include <mc_rtc/path.h>

namespace mc_control
{

namespace details
{

// FIXME This could be inside ControllerServerConfiguration but it crashes clang-10

/** "Classic" socket configuration
 *
 * \tparam default_pub_port Default publisher port
 *
 * \tparam default_pull_port Default pull port
 */
template<uint16_t default_pub_port, uint16_t default_pull_port>
struct SocketConfiguration
{
  /** Which host the socket binds to */
  std::string host = "*";
  /** Publisher port */
  uint16_t pub_port = default_pub_port;
  /** Pull request port */
  uint16_t pull_port = default_pull_port;
};

} // namespace details

/** Configuration for \ref mc_control::ControllerServer */
struct MC_CONTROL_DLLAPI ControllerServerConfiguration
{
  /** Controller server publication timestep
   *
   * If it is null or negative, the timestep will be the same as the controller
   *
   * If it happens to be lower than the controller's timestep then it will also be the same
   */
  double timestep = 0.05;

  /** IPC socket file
   *
   * Actual ipc sockets are created as socket + "_pub.ipc" and socket + "_rep.ipc"
   *
   * If nullopt, IPC is disabled
   */
  std::optional<std::string> ipc_socket = mc_rtc::temp_directory_path("mc_rtc");

  using TCPConfiguration = details::SocketConfiguration<4242, 4343>;

  /** Configuration for the TCP socket
   *
   * TCP is disabled if this is nullopt
   */
  std::optional<TCPConfiguration> tcp_config = TCPConfiguration{};

  using WebSocketConfiguration = details::SocketConfiguration<8080, 8081>;

  /** Configuration for the WebSocket socket
   *
   * WebSocket is disabled if this is nullopt (default)
   */
  std::optional<WebSocketConfiguration> websocket_config = std::nullopt;

  /** Loads from a configuration object */
  void load(const mc_rtc::Configuration & config);

  /** Returns the URI(s) the PUB socket should bind to */
  std::vector<std::string> pub_uris() const noexcept;

  /** Returns the URI(s) the PULL socket should bind to */
  std::vector<std::string> pull_uris() const noexcept;

  /** Prints a message about the server configuration */
  void print_serving_information() const noexcept;

  static ControllerServerConfiguration fromConfiguration(const mc_rtc::Configuration & config);
};

} // namespace mc_control
