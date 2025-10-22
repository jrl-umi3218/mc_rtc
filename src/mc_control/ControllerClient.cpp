/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/ControllerClient.h>

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/logging.h>

#ifndef MC_RTC_DISABLE_NETWORK
#  include <nanomsg/nn.h>
#  include <nanomsg/pipeline.h>
#  include <nanomsg/pubsub.h>
#  include <nanomsg/reqrep.h>
#endif

#include <chrono>
#include <sstream>
#include <stdexcept>
#include <thread>

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

} // namespace

namespace mc_control
{

#ifndef MC_RTC_DISABLE_NETWORK

namespace
{

void init_socket(int & socket, int proto, const std::string & uri, const std::string & name)
{
  socket = nn_socket(AF_SP, proto);
  if(socket < 0) { mc_rtc::log::error_and_throw("Failed to initialize {}", name); }
  int ret = nn_connect(socket, uri.c_str());
  if(ret < 0) { mc_rtc::log::error_and_throw("Failed to connect {} to uri: {}", name, uri); }
  else
  {
    mc_rtc::log::info("Connected {} to {}", name, uri);
  }
  if(proto == NN_SUB)
  {
    int err = nn_setsockopt(socket, NN_SUB, NN_SUB_SUBSCRIBE, "", 0);
    if(err < 0) { mc_rtc::log::error_and_throw("Failed to set subscribe option on SUB socket"); }
    int opt = -1;
    err = nn_setsockopt(socket, NN_SOL_SOCKET, NN_RCVMAXSIZE, &opt, sizeof(opt));
    if(err < 0) { mc_rtc::log::error_and_throw("Failed to set receive max size option on SUB socket"); }
  }
}

} // namespace

#endif

ControllerClient::ControllerClient() = default;

ControllerClient::ControllerClient(const std::string & sub_conn_uri, const std::string & push_conn_uri, double timeout)
: timeout_(timeout)
{
  connect(sub_conn_uri, push_conn_uri);
}

void ControllerClient::connect(const std::string & sub_conn_uri, const std::string & push_conn_uri)
{
#ifndef MC_RTC_DISABLE_NETWORK
  init_socket(sub_socket_, NN_SUB, sub_conn_uri, "SUB socket");
  init_socket(push_socket_, NN_PUSH, push_conn_uri, "PUSH socket");
  run_ = true;
#endif
}

ControllerClient::ControllerClient(ControllerServer & server, mc_rtc::gui::StateBuilder & gui)
{
  connect(server, gui);
}

void ControllerClient::connect(ControllerServer & server, mc_rtc::gui::StateBuilder & gui)
{
  server_ = &server;
  gui_ = &gui;
  run_ = true;
}

ControllerClient::~ControllerClient()
{
  stop();
}

void ControllerClient::stop()
{
  run_ = false;
#ifndef MC_RTC_DISABLE_NETWORK
  if(sub_th_.joinable()) { sub_th_.join(); }
  nn_shutdown(sub_socket_, 0);
  nn_shutdown(push_socket_, 0);
  sub_socket_ = -1;
  push_socket_ = -1;
#endif
  server_ = nullptr;
  gui_ = nullptr;
}

void ControllerClient::reconnect(const std::string & sub_conn_uri, const std::string & push_conn_uri)
{
  stop();
#ifndef MC_RTC_DISABLE_NETWORK
  init_socket(sub_socket_, NN_SUB, sub_conn_uri, "SUB socket");
  init_socket(push_socket_, NN_PUSH, push_conn_uri, "PUSH socket");
#endif
  start();
}

void ControllerClient::run(std::vector<char> & buff, std::chrono::system_clock::time_point & t_last_received)
{
  auto resize_to_fit = [&](size_t s)
  {
    if(buff.size() >= s) { return; }
    size_t nsize = buff.size() == 0 ? 65535 : 2 * buff.size();
    while(nsize < s) { nsize = 2 * nsize; }
    buff.resize(nsize);
  };
  if(sub_socket_ >= 0)
  {
#ifndef MC_RTC_DISABLE_NETWORK
    memset(buff.data(), 0, buff.size() * sizeof(char));
    auto recv = nn_recv(sub_socket_, buff.data(), buff.size(), NN_DONTWAIT);
    auto now = std::chrono::system_clock::now();
    if(recv < 0)
    {
      if(timeout_ > 0 && now - t_last_received > std::chrono::duration<double>(timeout_))
      {
        t_last_received = now;
        if(run_) { handle_gui_state(mc_rtc::Configuration{}); }
      }
      auto err = nn_errno();
      if(err != EAGAIN) { mc_rtc::log::error("ControllerClient failed to receive with errno: {}", err); }
    }
    else if(recv > 0)
    {
      auto msg_size = recv;
      while(recv > 0)
      {
        msg_size = recv;
        recv = nn_recv(sub_socket_, buff.data(), buff.size(), NN_DONTWAIT);
      }
      t_last_received = now;
      if(msg_size > static_cast<int>(buff.size()))
      {
        mc_rtc::log::warning(
            "Receive buffer was too small to receive the latest state message, will resize for next time");
        resize_to_fit(static_cast<size_t>(msg_size));
        return;
      }
      run(buff.data(), static_cast<size_t>(msg_size));
    }
#endif
  }
  else if(server_ != nullptr)
  {
    auto recv = server_->data();
    if(recv.second == 0) { return; }
    resize_to_fit(recv.second);
    memcpy(buff.data(), recv.first, recv.second * sizeof(char));
    run(buff.data(), recv.second);
  }
  else
  {
    handle_gui_state(mc_rtc::Configuration{});
  }
}

void ControllerClient::run(const char * buffer, size_t bufferSize)
{
  if(run_) { handle_gui_state(mc_rtc::Configuration::fromMessagePack(buffer, bufferSize)); }
}

void ControllerClient::start()
{
  run_ = true;
#ifndef MC_RTC_DISABLE_NETWORK
  sub_th_ = std::thread(
      [this]()
      {
        std::vector<char> buff(65536);
        auto t_last_received = std::chrono::system_clock::now();
        while(run_)
        {
          run(buff, t_last_received);
          std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
      });
#endif
}

void ControllerClient::send_request(const ElementId & id, const mc_rtc::Configuration & data)
{
  std::string out;
  raw_request(id, data, out);
#ifndef MC_RTC_DISABLE_NETWORK
  nn_send(push_socket_, out.c_str(), out.size() + 1, NN_DONTWAIT);
#endif
  if(server_) { server_->handle_requests(*gui_, out.c_str()); }
}

void ControllerClient::send_request(const ElementId & id)
{
  send_request(id, mc_rtc::Configuration{});
}

void ControllerClient::raw_request(const ElementId & id, const mc_rtc::Configuration & data, std::string & out)
{
  mc_rtc::Configuration request;
  request.add("category", id.category);
  request.add("name", id.name);
  request.add("data", data);
  out = request.dump();
}

void ControllerClient::raw_request(const ElementId & id, std::string & out)
{
  raw_request(id, mc_rtc::Configuration{}, out);
}

void ControllerClient::timeout(double t)
{
  timeout_ = t;
}

double ControllerClient::timeout()
{
  return timeout_;
}

void ControllerClient::handle_gui_state(mc_rtc::Configuration state)
{
  if(!state.size())
  {
    started();
    handle_category({}, "", {});
    stopped();
    return;
  }
  started();
  int version = state[0];
  if(version > mc_rtc::gui::StateBuilder::PROTOCOL_VERSION)
  {
    mc_rtc::log::error("Receive message, version: {} but I can only handle version {} and lower", version,
                       mc_rtc::gui::StateBuilder::PROTOCOL_VERSION);
    handle_category({}, "", {});
    stopped();
    return;
  }
  data_ = state[1];
  handle_category({}, "", state[2]);
  if(3 < state.size())
  {
    auto plots = state[3];
    for(size_t i = 0; i < plots.size(); ++i) { handle_plot(plots[i]); }
  }
  stopped();
}

void ControllerClient::handle_category(const std::vector<std::string> & parent,
                                       const std::string & category,
                                       const mc_rtc::Configuration & data)
{
  if(data.size() < 2) { return; }
  if(category.size()) { this->category(parent, category); }
  auto next_category = parent;
  if(category.size()) { next_category.push_back(category); }
  for(size_t i = 1; i < data.size() - 1; ++i)
  {
    auto widget_data = data[i];
    std::string widget_name = widget_data[0];
    int sid = widget_data.at(2, -1);
    handle_widget({next_category, widget_name, sid}, widget_data);
  }
  if(data[data.size() - 1].size())
  {
    auto cat_data = data[data.size() - 1];
    for(size_t i = 0; i < cat_data.size(); ++i) { handle_category(next_category, cat_data[i][0], cat_data[i]); }
  }
}

void ControllerClient::handle_widget(const ElementId & id, const mc_rtc::Configuration & data)
{
  auto type = static_cast<mc_rtc::gui::Elements>(static_cast<int>(data[1]));
  try
  {
    using Elements = mc_rtc::gui::Elements;
    switch(type)
    {
      case Elements::Label:
        label(id, std::string{data.at(3, data[3].dump())});
        break;
      case Elements::ArrayLabel:
        array_label(id, data.at(4, std::vector<std::string>{}), data[3]);
        break;
      case Elements::Button:
        button(id);
        break;
      case Elements::Checkbox:
        checkbox(id, data[3]);
        break;
      case Elements::StringInput:
        string_input(id, data[3]);
        break;
      case Elements::IntegerInput:
        integer_input(id, data[3]);
        break;
      case Elements::NumberInput:
        number_input(id, data[3]);
        break;
      case Elements::NumberSlider:
        number_slider(id, data[3], data[4], data[5]);
        break;
      case Elements::ArrayInput:
        array_input(id, data.at(4, std::vector<std::string>{}), data[3]);
        break;
      case Elements::ComboInput:
        combo_input(id, data[4], data[3]);
        break;
      case Elements::DataComboInput:
        data_combo_input(id, data[4], data[3]);
        break;
      case Elements::Point3D:
        handle_point3d(id, data);
        break;
      case Elements::Trajectory:
        handle_trajectory(id, data);
        break;
      case Elements::Polygon:
        handle_polygon(id, data);
        break;
      case Elements::PolyhedronTrianglesList:
        handle_polyhedron_triangles_list(id, data);
        break;
      case Elements::PolyhedronVerticesTriangles:
        handle_polyhedron_vertices_triangles(id, data);
        break;
      case Elements::Force:
        handle_force(id, data);
        break;
      case Elements::Arrow:
        handle_arrow(id, data);
        break;
      case Elements::Rotation:
        handle_rotation(id, data);
        break;
      case Elements::Transform:
        handle_transform(id, data);
        break;
      case Elements::Schema:
        schema(id, data[3]);
        break;
      case Elements::Form:
      {
        form(id);
        handle_form(id, data[3]);
        break;
      }
      case Elements::XYTheta:
        handle_xytheta(id, data);
        break;
      case Elements::Table:
        handle_table(id, data.at(3, std::vector<std::string>{}), data.at(5, std::vector<std::string>{}),
                     data.at(4, std::vector<mc_rtc::Configuration>{}));
        break;
      case Elements::Robot:
        robot(id, data[3], data[4], data[5]);
        break;
      case Elements::RobotMsg:
      {
        mc_rtc::gui::RobotMsgData msg(data[3], data[4], data[5], data[6], data[7], data[8], data[9]);
        robot_msg(id, msg);
        break;
      }
      case Elements::Visual:
        if(data[4].size() == 3)
        {
          Eigen::Vector3d pos = data[4];
          visual(id, data[3], {pos});
        }
        else
        {
          visual(id, data[3], data[4]);
        }
        break;
      default:
        mc_rtc::log::error("Type {} is not handlded by this ControllerClient", static_cast<int>(type));
        break;
    };
  }
  catch(const mc_rtc::Configuration::Exception & exc)
  {
    mc_rtc::log::error("Deserialization of GUI entry {} in category {} went wrong...", id.name, cat2str(id.category));
    mc_rtc::log::warning("Data was:\n{}", data.dump(true));
    mc_rtc::log::warning("mc_rtc::Configuration exception was:\n", exc.what());
  }
}

void ControllerClient::default_impl(const std::string & type, const ElementId & id)
{
  mc_rtc::log::warning("This implementation of ControllerClient does not handle {} GUI needed by {}/{}", type,
                       cat2str(id.category), id.name);
}

void ControllerClient::polyhedron(const ElementId & id,
                                  const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                                  const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors,
                                  const mc_rtc::gui::PolyhedronConfig & config)
{
  // The default other polyhedron implementation was called and this is also the default implementation, we are running
  // in circle
  if(default_polyhedron_vertices_triangles_)
  {
    default_impl("Polyhedron", id);
    return;
  }
  // Otherwise we transform this call for the other implementation
  default_polyhedron_triangles_list_ = true;
  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(3 * triangles.size());
  std::vector<std::array<size_t, 3>> indices;
  indices.reserve(triangles.size());
  for(size_t i = 0; i < triangles.size(); ++i)
  {
    vertices.push_back(triangles[i][0]);
    vertices.push_back(triangles[i][1]);
    vertices.push_back(triangles[i][2]);
    indices.push_back({3 * i, 3 * i + 1, 3 * i + 2});
  }
  std::vector<mc_rtc::gui::Color> vertices_colors;
  vertices_colors.reserve(3 * colors.size());
  for(const auto & c : colors)
  {
    vertices_colors.push_back(c[0]);
    vertices_colors.push_back(c[1]);
    vertices_colors.push_back(c[2]);
  }
  polyhedron(id, vertices, indices, vertices_colors, config);
}

void ControllerClient::polyhedron(const ElementId & id,
                                  const std::vector<Eigen::Vector3d> & vertices,
                                  const std::vector<std::array<size_t, 3>> & indices,
                                  const std::vector<mc_rtc::gui::Color> & colors,
                                  const mc_rtc::gui::PolyhedronConfig & config)
{
  // The default other polyhedron implementation was called and this is also the default implementation, we are running
  // in circle
  if(default_polyhedron_triangles_list_)
  {
    default_impl("Polyhedron", id);
    return;
  }
  // Otherwise we transform this call for the other implementation
  default_polyhedron_vertices_triangles_ = true;
  std::vector<std::array<Eigen::Vector3d, 3>> triangles;
  triangles.reserve(indices.size());
  for(const auto & idx : indices) { triangles.push_back({vertices[idx[0]], vertices[idx[1]], vertices[idx[2]]}); }
  std::vector<std::array<mc_rtc::gui::Color, 3>> triangle_colors;
  if(colors.size())
  {
    triangle_colors.reserve(indices.size());
    for(const auto & idx : indices) { triangle_colors.push_back({colors[idx[0]], colors[idx[1]], colors[idx[2]]}); }
  }
  polyhedron(id, triangles, triangle_colors, config);
}

void ControllerClient::handle_point3d(const ElementId & id, const mc_rtc::Configuration & data)
{
  Eigen::Vector3d pos = data[3];
  bool ro = data[4];
  mc_rtc::gui::PointConfig config;
  if(data.size() > 5) { config.fromMessagePack(data[5]); }
  if(ro) { array_label(id, {"x", "y", "z"}, pos); }
  else
  {
    array_input(id, {"x", "y", "z"}, pos);
  }
  point3d({id.category, id.name + "_point3d", id.sid}, id, ro, pos, config);
}

void ControllerClient::handle_trajectory(const ElementId & id, const mc_rtc::Configuration & data_)
{
  const auto & data = data_[3];
  mc_rtc::gui::LineConfig config;
  if(data_.size() > 4) { config.fromMessagePack(data_[4]); }
  if(data.isArray())
  {
    try
    {
      const std::vector<sva::PTransformd> & points = data;
      trajectory(id, points, config);
    }
    catch(mc_rtc::Configuration::Exception & exc)
    {
      exc.silence();
      try
      {
        const std::vector<Eigen::Vector3d> & points = data;
        trajectory(id, points, config);
      }
      catch(mc_rtc::Configuration::Exception & exc)
      {
        exc.silence();
        try
        {
          Eigen::Vector3d p = data;
          trajectory(id, p, config);
        }
        catch(mc_rtc::Configuration::Exception & exc)
        {
          exc.silence();
          sva::PTransformd pos = data;
          trajectory(id, pos, config);
        }
      }
    }
  }
  else
  {
    /** Default to dotted for real-time trajectories */
    sva::PTransformd pos = data;
    trajectory(id, pos, config);
  }
}

void ControllerClient::handle_polygon(const ElementId & id, const mc_rtc::Configuration & data_)
{
  const auto & data = data_[3];
  mc_rtc::gui::LineConfig config;
  if(data_.size() > 4)
  {
    if(data_[4].size() == 4)
    {
      config.color.fromMessagePack(data_[4]);
      config.width = 0.005;
    }
    else
    {
      config.fromMessagePack(data_[4]);
    }
  }
  try
  {
    const std::vector<std::vector<Eigen::Vector3d>> & points = data;
    polygon(id, points, config);
  }
  catch(mc_rtc::Configuration::Exception & exc)
  {
    exc.silence();
    try
    {
      std::vector<Eigen::Vector3d> p = data;
      polygon(id, {p}, config);
    }
    catch(mc_rtc::Configuration::Exception & exc)
    {
      mc_rtc::log::error("Could not deserialize polygon, supported data is vector<vector<Eigen::Vector3d>> or "
                         "vector<Eigen::Vector3d>");
      mc_rtc::log::error(exc.what());
      exc.silence();
    }
  }
}

void ControllerClient::handle_polyhedron_triangles_list(const ElementId & id, const mc_rtc::Configuration & data_)
{
  mc_rtc::gui::PolyhedronConfig config;
  config.fromMessagePack(data_[4]);

  std::vector<std::array<Eigen::Vector3d, 3>> triangles;
  std::vector<std::array<mc_rtc::gui::Color, 3>> colors;
  try
  {
    triangles = data_[3];
  }
  catch(mc_rtc::Configuration::Exception & exc)
  {
    mc_rtc::log::error(
        "Could not deserialize polyhedron vertices, supported data is vector<std::array<Eigen::Vector3d, 3>>");
    mc_rtc::log::error(exc.what());
    exc.silence();
    return;
  }
  if(data_.size() > 5)
  {
    try
    {
      colors = data_[5];
    }
    catch(mc_rtc::Configuration::Exception & exc)
    {
      mc_rtc::log::error(
          "Could not deserialize polyhedron colors, supported data is vector<std::array<mc_rtc::gui::Color,3>>");
      mc_rtc::log::error(exc.what());
      exc.silence();
    }
    if(colors.size() != 0 && colors.size() != triangles.size())
    {
      mc_rtc::log::error("{}/{} is not providing enough color data (expected: {}, got: {})", cat2str(id.category),
                         id.name, triangles.size(), colors.size());
      return;
    }
  }
  polyhedron(id, triangles, colors, config);
}

void ControllerClient::handle_polyhedron_vertices_triangles(const ElementId & id, const mc_rtc::Configuration & data_)
{
  mc_rtc::gui::PolyhedronConfig config;
  config.fromMessagePack(data_[5]);

  std::vector<Eigen::Vector3d> vertices;
  try
  {
    vertices = data_[3];
  }
  catch(mc_rtc::Configuration::Exception & exc)
  {
    mc_rtc::log::error("Could not deserialize polyhedron vertices, expecting a list of vertices");
    mc_rtc::log::error(exc.what());
    exc.silence();
    return;
  }
  std::vector<std::array<size_t, 3>> indices;
  try
  {
    indices = data_[4];
  }
  catch(mc_rtc::Configuration::Exception & exc)
  {
    mc_rtc::log::error("Could not deserialize polyhedron triangles, expecting a list of indices");
    mc_rtc::log::error(exc.what());
    exc.silence();
    return;
  }
  std::vector<mc_rtc::gui::Color> colors;
  if(data_.size() > 6)
  {
    try
    {
      colors = data_[6];
    }
    catch(mc_rtc::Configuration::Exception & exc)
    {
      mc_rtc::log::error("Could not deserialize polyhedron colors, supported data is std::vector<mc_rtc::gui::Color>");
      mc_rtc::log::error(exc.what());
      exc.silence();
    }
    if(colors.size() != 0 && colors.size() != vertices.size())
    {
      mc_rtc::log::error("{}/{} is not providing enough color data (expected: {}, got: {})", cat2str(id.category),
                         id.name, vertices.size(), colors.size());
      return;
    }
  }
  polyhedron(id, vertices, indices, colors, config);
}

void ControllerClient::handle_force(const ElementId & id, const mc_rtc::Configuration & data)
{
  const sva::ForceVecd & force_ = data[3];
  const sva::PTransformd & surface = data[4];
  bool ro = data[5];
  mc_rtc::gui::ForceConfig forceConfig;
  if(data.size() > 6) { forceConfig.fromMessagePack(data[6]); }
  if(ro) { array_label(id, {"cx", "cy", "cz", "fx", "fy", "fz"}, force_.vector()); }
  else
  {
    array_input(id, {"cx", "cy", "cz", "fx", "fy", "fz"}, force_.vector());
  }
  force({id.category, id.name + "_arrow", id.sid}, id, force_, surface, forceConfig, ro);
}

void ControllerClient::handle_arrow(const ElementId & id, const mc_rtc::Configuration & data)
{
  const Eigen::Vector3d & arrow_start = data[3];
  const Eigen::Vector3d & arrow_end = data[4];
  bool ro = data[5];
  mc_rtc::gui::ArrowConfig arrow_config;
  if(data.size() > 6) { arrow_config.fromMessagePack(data[6]); }
  Eigen::Vector6d arrow_data;
  arrow_data.head<3>() = arrow_start;
  arrow_data.tail<3>() = arrow_end;
  if(ro) { array_label(id, {"tx_0", "ty_0", "tz_0", "tx_1", "ty_1", "tz_1"}, arrow_data); }
  else
  {
    array_input(id, {"tx_0", "ty_0", "tz_0", "tx_1", "ty_1", "tz_1"}, arrow_data);
  }
  arrow({id.category, id.name + "_arrow", id.sid}, id, arrow_start, arrow_end, arrow_config, ro);
}

void ControllerClient::handle_rotation(const ElementId & id, const mc_rtc::Configuration & data)
{
  sva::PTransformd pos = data[3];
  bool ro = data[4];
  Eigen::Quaterniond q{pos.rotation()};
  if(ro) { array_label(id, {"w", "x", "y", "z"}, Eigen::Vector4d{q.w(), q.x(), q.y(), q.z()}); }
  else
  {
    array_input(id, {"w", "x", "y", "z"}, Eigen::Vector4d{q.w(), q.x(), q.y(), q.z()});
  }
  rotation({id.category, id.name + "_rotation", id.sid}, id, ro, pos);
}

void ControllerClient::handle_transform(const ElementId & id, const mc_rtc::Configuration & data)
{
  const sva::PTransformd & transform_ = data[3];
  bool ro = data[4];

  auto publish_transform = [this](const sva::PTransformd & pos, const ElementId & id, bool ro)
  {
    Eigen::Quaterniond q{pos.rotation()};
    Eigen::Matrix<double, 7, 1> vec;
    vec << q.w(), q.x(), q.y(), q.z(), pos.translation();
    if(ro) { array_label(id, {"qw", "qx", "qy", "qz", "tx", "ty", "tz"}, vec); }
    else
    {
      array_input(id, {"qw", "qx", "qy", "qz", "tx", "ty", "tz"}, vec);
    }
    transform({id.category, id.name + "_transform", id.sid}, id, ro, pos);
  };

  publish_transform(transform_, id, ro);
}

void ControllerClient::handle_xytheta(const ElementId & id, const mc_rtc::Configuration & data)
{
  Eigen::VectorXd vec = data[3];
  bool ro = data[4];
  if(vec.size() < 3)
  {
    mc_rtc::log::error(
        "Could not deserialize xytheta element. Expected VectorXd of size 3 or 4 (x, y, theta, [altitude])");
    return;
  }

  Eigen::Vector3d xythetaVec = vec.head<3>();
  double altitude = 0;
  if(vec.size() == 4) { altitude = vec(3); }

  const std::vector<std::string> & label = {"X", "Y", "Theta", "Altitude"};
  if(ro) { array_label(id, label, vec); }
  else
  {
    array_input(id, label, vec);
  }
  xytheta({id.category, id.name + "_xytheta", id.sid}, id, ro, xythetaVec, altitude);
}

void ControllerClient::handle_form(const ElementId & id, const mc_rtc::Configuration & gui)
{
  for(size_t i = 0; i < gui.size(); ++i)
  {
    auto el = gui[i];
    std::string name = el[0];
    auto type = static_cast<mc_rtc::gui::Elements>(static_cast<int>(el[1]));
    bool required = el[2];
    using Elements = mc_rtc::gui::Elements;
    switch(type)
    {
      case Elements::Checkbox:
        form_checkbox(id, name, required, el[3], el.size() > 4 ? el[4] : true);
        break;
      case Elements::IntegerInput:
        form_integer_input(id, name, required, el[3], el.size() > 4 ? el[4] : true);
        break;
      case Elements::NumberInput:
        form_number_input(id, name, required, el[3], el.size() > 4 ? el[4] : true);
        break;
      case Elements::StringInput:
        form_string_input(id, name, required, el[3], el.size() > 4 ? el[4] : true);
        break;
      case Elements::ArrayInput:
        form_array_input(id, name, required, el.size() > 6 ? el[6] : std::vector<std::string>{}, el[3], el[4],
                         el.size() > 5 ? el[5] : true);
        break;
      case Elements::ComboInput:
        form_combo_input(id, name, required, el[3], el[4], el.size() > 5 ? el[5] : -1);
        break;
      case Elements::DataComboInput:
        form_data_combo_input(id, name, required, el[3], el[4]);
        break;
      case Elements::Point3D:
        form_point3d_input(id, name, required, el[3], el[4], el[5]);
        break;
      case Elements::Rotation:
        form_rotation_input(id, name, required, el[3], el[4], el[5]);
        break;
      case Elements::Transform:
        form_transform_input(id, name, required, el[3], el[4], el[5]);
        break;
      case Elements::Form:
      {
        start_form_object_input(name, required);
        handle_form(id, el[3]);
        end_form_object_input();
        break;
      }
      case Elements::GenericArray:
      {
        if(el[3].size() != 1)
        {
          mc_rtc::log::error("GenericArray ({}) has more than one element describing its content!", name);
          break;
        }
        std::optional<std::vector<Configuration>> data = std::nullopt;
        if(el[4].isArray()) { data = el[4].operator std::vector<Configuration>(); }
        start_form_generic_array_input(name, required, data);
        handle_form(id, el[3]);
        end_form_generic_array_input();
        break;
      }
      case Elements::OneOf:
      {
        std::optional<std::pair<size_t, Configuration>> data = std::nullopt;
        if(el[3].isArray()) { data = el[3].operator std::pair<size_t, Configuration>(); }
        start_form_one_of_input(name, required, data);
        handle_form(id, el[4]);
        end_form_one_of_input();
        break;
      }
      default:
        mc_rtc::log::error("Form cannot handle element of type {}", static_cast<int>(type));
    }
  }
}

void ControllerClient::handle_plot(const mc_rtc::Configuration & plot)
{
  auto pType = static_cast<mc_rtc::gui::plot::Plot>(static_cast<uint64_t>(plot[0]));
  switch(pType)
  {
    case mc_rtc::gui::plot::Plot::Standard:
      handle_standard_plot(plot);
      break;
    case mc_rtc::gui::plot::Plot::XY:
      handle_xy_plot(plot);
      break;
    default:
      mc_rtc::log::error("This client implementation only handles standard and XY plots");
  }
}

namespace
{

struct X
{
  mc_rtc::gui::plot::AxisConfiguration config;
  std::vector<double> values;

  X(const mc_rtc::Configuration & data)
  {
    config.fromMessagePack(data[0]);
    values = data[1];
  }
};

struct Y
{
  std::string legend;
  std::vector<double> values;
  mc_rtc::gui::Color color;
  mc_rtc::gui::plot::Style style;
  mc_rtc::gui::plot::Side side;

  Y(const mc_rtc::Configuration & data)
  {
    legend = static_cast<std::string>(data[1]);
    values = data[2];
    color.fromMessagePack(data[3]);
    style = static_cast<mc_rtc::gui::plot::Style>(static_cast<uint64_t>(data[4]));
    side = static_cast<mc_rtc::gui::plot::Side>(static_cast<uint64_t>(data[5]));
  }
};

struct XY
{
  std::string legend;
  std::vector<std::array<double, 2>> values;
  mc_rtc::gui::Color color;
  mc_rtc::gui::plot::Style style;
  mc_rtc::gui::plot::Side side;

  XY(const mc_rtc::Configuration & data)
  {
    legend = static_cast<std::string>(data[1]);
    values = data[2];
    color.fromMessagePack(data[3]);
    style = static_cast<mc_rtc::gui::plot::Style>(static_cast<uint64_t>(data[4]));
    side = static_cast<mc_rtc::gui::plot::Side>(static_cast<uint64_t>(data[5]));
  }
};

struct Polygon
{
  std::string legend;
  mc_rtc::gui::plot::PolygonDescription polygon;
  mc_rtc::gui::plot::Side side;

  Polygon(const mc_rtc::Configuration & data)
  {
    legend = static_cast<std::string>(data[1]);
    polygon.fromMessagePack(data[2]);
    side = static_cast<mc_rtc::gui::plot::Side>(static_cast<uint64_t>(data[3]));
  }
};

struct Polygons
{
  std::string legend;
  std::vector<mc_rtc::gui::plot::PolygonDescription> polygons;
  mc_rtc::gui::plot::Side side;

  Polygons(const mc_rtc::Configuration & data)
  {
    legend = static_cast<std::string>(data[1]);
    for(size_t i = 0; i < data[2].size(); ++i)
    {
      polygons.emplace_back();
      polygons.back().fromMessagePack(data[2][i]);
    }
    side = static_cast<mc_rtc::gui::plot::Side>(static_cast<uint64_t>(data[3]));
  }
};

} // namespace

void ControllerClient::handle_standard_plot(const mc_rtc::Configuration & plot)
{
  uint64_t id = plot[1];
  std::string title = plot[2];
  start_plot(id, title);
  X x(plot[3]);
  plot_setup_xaxis(id, x.config.name, x.config.range);
  mc_rtc::gui::plot::AxisConfiguration y1Config;
  y1Config.fromMessagePack(plot[4]);
  plot_setup_yaxis_left(id, y1Config.name, y1Config.range);
  mc_rtc::gui::plot::AxisConfiguration y2Config;
  y2Config.fromMessagePack(plot[5]);
  plot_setup_yaxis_right(id, y2Config.name, y2Config.range);
  for(size_t i = 6; i < plot.size(); ++i)
  {
    const auto & y_ = plot[i];
    auto type = static_cast<mc_rtc::gui::plot::Type>(static_cast<uint64_t>(y_[0]));
    using Type = mc_rtc::gui::plot::Type;
    if(type == Type::Ordinate)
    {
      Y y(y_);
      if(x.values.size() < y.values.size())
      {
        mc_rtc::log::error("[Plot::{}] Not enough X data compared to Y data", title);
      }
      size_t x_0 = y.values.size() - x.values.size();
      for(size_t j = 0; j < y.values.size(); ++j)
      {
        plot_point(id, i - 6, y.legend, x.values[x_0 + j], y.values[j], y.color, y.style, y.side);
      }
    }
    else if(type == Type::AbscissaOrdinate)
    {
      XY xy(y_);
      for(const auto & v : xy.values) { plot_point(id, i - 6, xy.legend, v[0], v[1], xy.color, xy.style, xy.side); }
    }
    else if(type == Type::Polygon)
    {
      Polygon polygon(y_);
      plot_polygon(id, i - 6, polygon.legend, polygon.polygon, polygon.side);
    }
    else if(type == Type::Polygons)
    {
      Polygons polygons(y_);
      plot_polygons(id, i - 6, polygons.legend, polygons.polygons, polygons.side);
    }
    else
    {
      mc_rtc::log::error("Cannot handle provided data in {}:", title);
      mc_rtc::log::warning(y_.dump(true, true));
    }
  }
  end_plot(id);
}

void ControllerClient::handle_xy_plot(const mc_rtc::Configuration & plot)
{
  uint64_t id = plot[1];
  std::string title = plot[2];
  start_plot(id, title);
  mc_rtc::gui::plot::AxisConfiguration xConfig;
  xConfig.fromMessagePack(plot[3]);
  plot_setup_xaxis(id, xConfig.name, xConfig.range);
  mc_rtc::gui::plot::AxisConfiguration y1Config;
  y1Config.fromMessagePack(plot[4]);
  plot_setup_yaxis_left(id, y1Config.name, y1Config.range);
  mc_rtc::gui::plot::AxisConfiguration y2Config;
  y2Config.fromMessagePack(plot[5]);
  plot_setup_yaxis_right(id, y2Config.name, y2Config.range);
  for(size_t i = 6; i < plot.size(); ++i)
  {
    const auto & y_ = plot[i];
    auto type = static_cast<mc_rtc::gui::plot::Type>(static_cast<uint64_t>(y_[0]));
    using Type = mc_rtc::gui::plot::Type;
    if(type == Type::AbscissaOrdinate)
    {
      XY xy(y_);
      for(const auto & v : xy.values) { plot_point(id, i - 6, xy.legend, v[0], v[1], xy.color, xy.style, xy.side); }
    }
    else if(type == Type::Polygon)
    {
      Polygon polygon(y_);
      plot_polygon(id, i - 6, polygon.legend, polygon.polygon, polygon.side);
    }
    else if(type == Type::Polygons)
    {
      Polygons polygons(y_);
      plot_polygons(id, i - 6, polygons.legend, polygons.polygons, polygons.side);
    }
    else
    {
      mc_rtc::log::error("Cannot handle provided data in {}:", title);
      mc_rtc::log::warning(y_.dump(true, true));
    }
  }
  end_plot(id);
}

void ControllerClient::handle_table(const ElementId & id,
                                    const std::vector<std::string> & header,
                                    const std::vector<std::string> & format,
                                    const std::vector<mc_rtc::Configuration> & data)
{
  auto format_data = [&](const mc_rtc::Configuration & entry, size_t i)
  {
    const auto & f = i < format.size() ? format[i] : "{}";
    if(entry[i].isString())
    {
      try
      {
        return fmt::format(f, entry.at(i, entry[i].dump()));
      }
      catch(const fmt::format_error & error)
      {
        mc_rtc::log::warning("Failed to format data ({}) with the given format ({}) (error: {})", entry[i].dump(), f,
                             error.what());
      }
    }
    else
    {
      try
      {
        return fmt::format(f, static_cast<double>(entry[i]));
      }
      catch(mc_rtc::Configuration::Exception & exc)
      {
        exc.silence();
        try
        {
          return fmt::format(f, entry[i].dump());
        }
        catch(const fmt::format_error & error)
        {
          mc_rtc::log::warning("Failed to format data ({}) with the given format ({}) (error: {})", entry[i].dump(), f,
                               error.what());
        }
      }
      catch(const fmt::format_error & error)
      {
        mc_rtc::log::warning("Failed to format data ({}) with the given format ({}) (error: {})", entry[i].dump(), f,
                             error.what());
      }
    }
    return entry.at(i, entry[i].dump());
  };
  table_start(id, header);
  std::vector<std::string> data_str;
  for(size_t i = 0; i < data.size(); ++i)
  {
    const auto & c = data[i];
    auto c_size = c.size();
    if(!c_size)
    {
      mc_rtc::log::error("Table {} (category: {}) is excepting an array of array in data", id.name,
                         cat2str(id.category));
      continue;
    }
    data_str.resize(c_size);
    for(size_t j = 0; j < c_size; ++j) { data_str[j] = format_data(c, j); }
    table_row(id, data_str);
  }
  table_end(id);
}

} // namespace mc_control
