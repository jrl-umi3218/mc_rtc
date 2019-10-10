/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/ControllerServer.h>

#include <nanomsg/nn.h>
#include <nanomsg/pipeline.h>
#include <nanomsg/pubsub.h>

namespace mc_control
{

ControllerServer::ControllerServer(double dt,
                                   double server_dt,
                                   const std::vector<std::string> & pub_bind_uri,
                                   const std::vector<std::string> & pull_bind_uri)
{
  iter_ = 0;
  rate_ = static_cast<unsigned int>(ceil(server_dt / dt));
  auto init_socket = [](int & socket, unsigned int proto, const std::vector<std::string> & uris,
                        const std::string & name) {
    socket = nn_socket(AF_SP, proto);
    if(socket < 0)
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "Failed to initialize " << name)
    }
    for(const auto & uri : uris)
    {
      int ret = nn_bind(socket, uri.c_str());
      if(ret < 0)
      {
        LOG_ERROR_AND_THROW(std::runtime_error, "Failed to bind " << name << " to uri: " << uri)
      }
    }
  };
  init_socket(pub_socket_, NN_PUB, pub_bind_uri, "PUB socket");
  init_socket(pull_socket_, NN_PULL, pull_bind_uri, "PULL socket");
}

ControllerServer::~ControllerServer()
{
  nn_shutdown(pub_socket_, 0);
  nn_shutdown(pull_socket_, 0);
}

void ControllerServer::handle_requests(mc_rtc::gui::StateBuilder & gui_builder)
{
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
        LOG_ERROR("ControllerServer failed to receive requested with errno: " << err)
      }
    }
    else
    {
      auto config = mc_rtc::Configuration::fromData(static_cast<const char *>(buf));
      auto category = config("category", std::vector<std::string>{});
      auto name = config("name", std::string{});
      auto data = config("data", mc_rtc::Configuration{});
      if(!gui_builder.handleRequest(category, name, data))
      {
        LOG_ERROR("Invokation of the following method failed" << std::endl << config.dump(true) << std::endl)
      }
      nn_freemsg(buf);
    }
  } while(recv > 0);
}

void ControllerServer::publish(mc_rtc::gui::StateBuilder & gui_builder)
{
  if(iter_++ % rate_ == 0)
  {
    auto s = gui_builder.update(buffer_);
    nn_send(pub_socket_, buffer_.data(), s, 0);
  }
}

} // namespace mc_control
