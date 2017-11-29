#include <mc_control/ControllerClient.h>

#include <mc_rtc/logging.h>

#include <stdexcept>
#include <sstream>

#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/reqrep.h>

namespace mc_control
{

ControllerClient::ControllerClient(const std::string & sub_conn_uri,
                                   const std::string & req_conn_uri)
{
  auto init_socket = [](int & socket,
                        unsigned int proto,
                        const std::string & uri,
                        const std::string & name)
  {
    socket = nn_socket(AF_SP, proto);
    if(socket < 0)
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "Failed to initialize " << name)
    }
    int ret = nn_connect(socket, uri.c_str());
    if(ret < 0)
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "Failed to connect " << name << " to uri: " << uri)
    }
    else
    {
      LOG_INFO("Connected " << name << " to " << uri)
    }
  };
  init_socket(sub_socket_, NN_SUB, sub_conn_uri, "SUB socket");
  int err = nn_setsockopt(sub_socket_, NN_SUB, NN_SUB_SUBSCRIBE, "", 0);
  if(err < 0)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Failed to set subscribe option on SUB socket")
  }
  init_socket(req_socket_, NN_REQ, req_conn_uri, "REQ socket");

  sub_th_ = std::thread([this]()
  {
  std::vector<char> buff(65536);
  while(run_)
  {
    auto recv = nn_recv(sub_socket_, buff.data(), buff.size(), 0);
    if(recv < 0)
    {
      auto err = nn_errno();
      if(err != EAGAIN)
      {
        LOG_ERROR("ControllerClient failed to receive with errno: " << err)
      }
    }
    else if(recv > static_cast<int>(buff.size()))
    {
      LOG_WARNING("Receive buffer was too small to receive the latest state message, will resize for next time")
      buff.resize(2*buff.size());
    }
    else if(recv > 0)
    {
      handle_gui_state(buff.data(), recv);
    }
  }
  });
}

ControllerClient::~ControllerClient()
{
  run_ = false;
  if(sub_th_.joinable())
  {
    sub_th_.join();
  }
  nn_shutdown(sub_socket_, 0);
  nn_shutdown(req_socket_, 0);
}

}
