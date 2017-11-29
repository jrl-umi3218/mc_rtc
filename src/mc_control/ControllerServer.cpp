#include <mc_control/ControllerServer.h>

#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/reqrep.h>

namespace mc_control
{

ControllerServer::ControllerServer(double dt, double server_dt,
                                   const std::vector<std::string> & pub_bind_uri,
                                   const std::vector<std::string> & rep_bind_uri)
{
  iter = 0;
  rate = ceil(server_dt / dt);
  auto init_socket = [](int & socket,
                        unsigned int proto,
                        const std::vector<std::string> & uris,
                        const std::string & name)
  {
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
  init_socket(rep_socket_, NN_REP, rep_bind_uri, "REP socket");
}

ControllerServer::~ControllerServer()
{
  nn_shutdown(pub_socket_, 0);
  nn_shutdown(rep_socket_, 0);
}

void ControllerServer::handle_requests()
{
  /*FIXME Implement */
}

void ControllerServer::publish(mc_rtc::gui::StateBuilder & gui_builder)
{
  if(iter++ % rate == 0)
  {
    const auto & state = gui_builder.updateState();
    auto data = state.state.dump();
    nn_send(pub_socket_, data.c_str(), data.size() + 1, 0);
  }
}

}
