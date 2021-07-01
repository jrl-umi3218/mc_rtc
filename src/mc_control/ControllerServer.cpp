/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/ControllerServer.h>

#ifndef MC_RTC_DISABLE_NETWORK
#  include <nanomsg/nn.h>
#  include <nanomsg/pipeline.h>
#  include <nanomsg/pubsub.h>
#  include <nanomsg/reqrep.h>
#endif

namespace mc_control
{

ControllerServer::ControllerServer(double dt,
                                   double server_dt,
                                   const std::vector<std::string> & pub_bind_uri,
                                   const std::vector<std::string> & pull_bind_uri)
{
  iter_ = 0;
  rate_ = static_cast<unsigned int>(ceil(server_dt / dt));
#ifndef MC_RTC_DISABLE_NETWORK
  auto init_socket = [](int & socket, int proto, const std::vector<std::string> & uris, const std::string & name) {
    socket = nn_socket(AF_SP, proto);
    if(socket < 0)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Failed to initialize {}", name);
    }
    for(const auto & uri : uris)
    {
      int ret = nn_bind(socket, uri.c_str());
      if(ret < 0)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("Failed to bind {} to uri: {}", name, uri);
      }
    }
  };
  init_socket(pub_socket_, NN_PUB, pub_bind_uri, "PUB socket");
  init_socket(pull_socket_, NN_PULL, pull_bind_uri, "PULL socket");
#endif
}

ControllerServer::~ControllerServer()
{
#ifndef MC_RTC_DISABLE_NETWORK
  nn_close(pub_socket_);
  nn_close(pull_socket_);
#endif
}

void ControllerServer::handle_requests(mc_rtc::gui::StateBuilder & gui_builder, const char * dataIn)
{
  auto config = mc_rtc::Configuration::fromData(static_cast<const char *>(dataIn));
  auto category = config("category", std::vector<std::string>{});
  auto name = config("name", std::string{});
  auto data = config("data", mc_rtc::Configuration{});
  if(!gui_builder.handleRequest(category, name, data))
  {
    mc_rtc::log::error("Invokation of the following method failed\n{}\n", config.dump(true));
  }
}

void ControllerServer::handle_requests(mc_rtc::gui::StateBuilder & gui_builder)
{
#ifndef MC_RTC_DISABLE_NETWORK
  /*FIXME Avoid freeing the message constantly */
  void * buf = nullptr;
  int recv = 0;
  do
  {
    recv = nn_recv(pull_socket_, &buf, NN_MSG, NN_DONTWAIT);
    if(recv < 0)
    {
      auto err = nn_errno();
      if(err != EAGAIN)
      {
        mc_rtc::log::error("ControllerServer failed to receive requested with errno: {}", err);
      }
    }
    else
    {
      handle_requests(gui_builder, static_cast<const char *>(buf));
      nn_freemsg(buf);
    }
  } while(recv > 0);
#endif
}

void ControllerServer::publish(mc_rtc::gui::StateBuilder & gui_builder)
{
  if(iter_++ % rate_ == 0)
  {
    buffer_size_ = gui_builder.update(buffer_);
#ifndef MC_RTC_DISABLE_NETWORK
    nn_send(pub_socket_, buffer_.data(), buffer_size_, 0);
#endif
  }
  else
  {
    buffer_size_ = 0;
  }
}

std::pair<const char *, size_t> ControllerServer::data() const
{
  return {buffer_.data(), buffer_size_};
}

} // namespace mc_control
