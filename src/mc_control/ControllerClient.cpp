#include <mc_control/ControllerClient.h>

#include <mc_rtc/GUIState.h>
#include <mc_rtc/logging.h>

#include <stdexcept>
#include <sstream>

#include <nanomsg/nn.h>
#include <nanomsg/pipeline.h>
#include <nanomsg/pubsub.h>

#include <unistd.h>

namespace
{

std::string cat2str(const std::vector<std::string> & cat)
{
  std::string ret;
  for(size_t i = 0; i < cat.size(); ++i)
  {
    ret += cat[i];
    if(i != cat.size() - 1) { ret += "/"; }
  }
  return ret;
}

}

namespace mc_control
{

ControllerClient::ControllerClient(const std::string & sub_conn_uri,
                                   const std::string & push_conn_uri,
                                   double timeout)
: timeout_(timeout)
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
  init_socket(push_socket_, NN_PUSH, push_conn_uri, "PUSH socket");

  sub_th_ = std::thread([this]()
  {
  std::vector<char> buff(65536);
  auto t_last_received = std::chrono::system_clock::now();
  while(run_)
  {
    auto recv = nn_recv(sub_socket_, buff.data(), buff.size(), NN_DONTWAIT);
    auto now = std::chrono::system_clock::now();
    if(recv < 0)
    {
      if(timeout_ > 0 && now - t_last_received > std::chrono::duration<double>(timeout_))
      {
        t_last_received = now;
        handle_gui_state("{}");
      }
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
      t_last_received = now;
      handle_gui_state(buff.data());
    }
    usleep(500);
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
  nn_shutdown(push_socket_, 0);
}

void ControllerClient::send_request(const std::vector<std::string> & category,
                                    const std::string & name,
                                    const mc_rtc::Configuration & data)
{
  mc_rtc::Configuration request;
  request.add("category", category);
  request.add("name", name);
  request.add("data", data);
  std::string out = request.dump();
  nn_send(push_socket_, out.c_str(), out.size() + 1, NN_DONTWAIT);
}

void ControllerClient::handle_gui_state(const char * data)
{
  started();
  auto state = mc_rtc::Configuration::fromData(data);
  if(state.has("DATA"))
  {
    data_.load(state("DATA"));
    state.remove("DATA");
  }
  else
  {
    data_ = mc_rtc::Configuration{};
  }
  handle_category({}, "", state);
  stopped();
}

void ControllerClient::handle_category(const std::vector<std::string> & parent,
                                       const std::string & category,
                                       const mc_rtc::Configuration & data)
{
  if(category.size())
  {
    this->category(parent, category);
  }
  auto next_category = parent;
  if(category.size())
  {
    next_category.push_back(category);
  }
  for(const auto & k : data.keys())
  {
    const auto & d = data(k);
    if(d.has("GUI"))
    {
      handle_widget(next_category, k, d);
    }
    else
    {
      handle_category(next_category, k, d);
    }
  }
}

void ControllerClient::handle_widget(const std::vector<std::string> & category,
                                     const std::string & name,
                                     const mc_rtc::Configuration & data)
{
  auto gui = data("GUI");
  if(!gui.has("type"))
  {
    LOG_ERROR("GUI entry " << name << " in category " << cat2str(category) << " has no type")
    return;
  }
  auto type = static_cast<mc_rtc::gui::Elements>(static_cast<int>(gui("type")));
  try
  {
    using Elements = mc_rtc::gui::Elements;
    switch(type)
    {
      case Elements::Label:
        label(category, name, data("data").dump());
        break;
      case Elements::ArrayLabel:
        array_label(category, name, gui("labels", std::vector<std::string>{}), data("data"));
        break;
      case Elements::Button:
        button(category, name);
        break;
      case Elements::Checkbox:
        checkbox(category, name, data("data"));
        break;
      default:
        LOG_ERROR("Type " << static_cast<int>(type) << " is not handlded by this ControllerClient")
        break;
    };
  }
  catch(const mc_rtc::Configuration::Exception & exc)
  {
    LOG_ERROR("Deserialization of GUI entry " << name << " in category " << cat2str(category) << " went wrong...")
    LOG_WARNING("Data was: " << std::endl << data.dump(true))
    LOG_WARNING("mc_rtc::Configuration exception was: " << std::endl << exc.what())
  }
}

void ControllerClient::default_impl(const std::string & type,
                                    const std::vector<std::string> & category,
                                    const std::string & label)
{
  LOG_WARNING("This implementation of ControllerClient does not handle " << type << " GUI needed by " << cat2str(category) << "/" << label)
}

}
