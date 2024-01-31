/*
 * Copyright 2015-2024 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/ControllerServerConfiguration.h>

namespace mc_control
{

ControllerServerConfiguration ControllerServerConfiguration::fromConfiguration(const mc_rtc::Configuration & config)
{
  ControllerServerConfiguration out;
  out.load(config);
  return out;
}

void ControllerServerConfiguration::load(const mc_rtc::Configuration & config)
{
  config("Timestep", timestep);
  if(auto ipc = config.find("IPC")) { (*ipc)("Socket", ipc_socket); }
  else { ipc_socket = std::nullopt; }
  auto socket_config = [&](const std::string & section, auto & opt_out)
  {
    using SocketT = typename std::remove_reference_t<decltype(opt_out)>::value_type;
    if(auto cfg = config.find(section))
    {
      opt_out = SocketT{};
      (*cfg)("Host", opt_out->host);
      if(auto ports_cfg = cfg->find("Ports"))
      {
        auto ports = ports_cfg->operator std::array<uint16_t, 2>();
        opt_out->pub_port = ports[0];
        opt_out->pull_port = ports[1];
      }
    }
    else { opt_out = std::nullopt; }
  };
  socket_config("TCP", tcp_config);
  socket_config("WS", websocket_config);
}

std::vector<std::string> ControllerServerConfiguration::pub_uris() const noexcept
{
  std::vector<std::string> uris;
  if(ipc_socket) { uris.push_back("ipc://" + *ipc_socket + "_pub.ipc"); }
  auto handle_socket = [&](const std::string & protocol, const auto & cfg)
  {
    if(!cfg) { return; }
    uris.push_back(protocol + cfg->host + ":" + std::to_string(cfg->pub_port));
  };
  handle_socket("tcp://", tcp_config);
  handle_socket("ws://", websocket_config);
  return uris;
}

std::vector<std::string> ControllerServerConfiguration::pull_uris() const noexcept
{
  std::vector<std::string> uris;
  if(ipc_socket) { uris.push_back("ipc://" + *ipc_socket + "_rep.ipc"); }
  auto handle_socket = [&](const std::string & protocol, const auto & cfg)
  {
    if(!cfg) { return; }
    uris.push_back(protocol + cfg->host + ":" + std::to_string(cfg->pull_port));
  };
  handle_socket("tcp://", tcp_config);
  handle_socket("ws://", websocket_config);
  return uris;
}

void ControllerServerConfiguration::print_serving_information() const noexcept
{
  mc_rtc::log::info("Publishing data on:");
  for(const auto & pub_uri : pub_uris()) { mc_rtc::log::info("- {}", pub_uri); }
  mc_rtc::log::info("Handling requests on:");
  for(const auto & pull_uri : pull_uris()) { mc_rtc::log::info("- {}", pull_uri); }
}

} // namespace mc_control
