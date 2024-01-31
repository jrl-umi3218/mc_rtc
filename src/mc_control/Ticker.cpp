/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/Ticker.h>

#include <thread>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_control
{

template<typename StrT>
std::string log_entry(const StrT & entry, const std::string & robot, bool is_main)
{
  if(is_main) { return entry; }
  return fmt::format("{}_{}", robot, entry);
}

/** Get a log entry for a robot from the given log at the given time */
template<typename GetT = std::vector<double>>
GetT get(const mc_rtc::log::FlatLog & log,
         const std::string & entry,
         const std::string & robot,
         bool is_main,
         size_t idx,
         const GetT & def = {})
{
  return log.get<GetT>(log_entry(entry, robot, is_main), idx, def);
}

/** Get the encoders of a robot from the given log at the given time */
std::vector<double> get_encoders(const mc_rtc::log::FlatLog & log, const std::string & robot, bool is_main, size_t idx)
{
  return get(log, "qIn", robot, is_main, idx);
}

/** Get the encoders of a robot from the robot's configuration */
std::vector<double> get_encoders(const mc_rbdyn::Robot & robot)
{
  std::vector<double> q;
  q.reserve(robot.refJointOrder().size());
  for(size_t i = 0; i < robot.refJointOrder().size(); ++i)
  {
    auto mbcIdx = robot.jointIndexInMBC(i);
    if(mbcIdx == -1)
    {
      q.push_back(0.0);
      continue;
    }
    for(const auto & qi : robot.mbc().q[static_cast<size_t>(mbcIdx)]) { q.push_back(qi); }
  }
  return q;
}

/** Get the encoders' velocities of a robot from the robot's configuration */
std::vector<double> get_encoders_velocities(const mc_rbdyn::Robot & robot)
{
  std::vector<double> alpha;
  alpha.reserve(robot.refJointOrder().size());
  for(size_t i = 0; i < robot.refJointOrder().size(); ++i)
  {
    auto mbcIdx = robot.jointIndexInMBC(i);
    if(mbcIdx == -1)
    {
      alpha.push_back(0.0);
      continue;
    }
    for(const auto & qi : robot.mbc().alpha[static_cast<size_t>(mbcIdx)]) { alpha.push_back(qi); }
  }
  return alpha;
}

/** Get the floating base position from the given log at the given time */
std::optional<sva::PTransformd> get_posW(const mc_rtc::log::FlatLog & log,
                                         const std::string & robot,
                                         bool is_main,
                                         size_t idx)
{
  if(log.meta())
  {
    auto it = log.meta()->init.find(robot);
    if(it != log.meta()->init.end()) { return it->second; }
  }
  auto entry = log_entry("ff", robot, is_main);
  if(!log.has(entry)) { return std::nullopt; }
  return log.get(entry, idx, sva::PTransformd::Identity());
}

std::optional<std::vector<double>> get_initial_encoders(const mc_rtc::log::FlatLog & log,
                                                        const mc_rbdyn::Robot & robot,
                                                        bool is_main)
{
  if(log.meta())
  {
    auto it = log.meta()->init_q.find(robot.name());
    if(it != log.meta()->init_q.end())
    {
      const auto & q = it->second;
      std::vector<double> out;
      for(size_t i = 0; i < robot.refJointOrder().size(); ++i)
      {
        auto mbcIdx = robot.jointIndexInMBC(i);
        if(mbcIdx >= 0 && !q[static_cast<size_t>(mbcIdx)].empty()) { out.push_back(q[static_cast<size_t>(mbcIdx)][0]); }
        else { out.push_back(0.0); }
      }
      return out;
    }
  }
  if(log.has(log_entry("qIn", robot.name(), is_main))) { return get_encoders(log, robot.name(), is_main, 0); }
  return std::nullopt;
}

/** Get the initial state of all robots from the given log */
auto get_initial_state(const mc_rtc::log::FlatLog & log, const mc_rbdyn::Robots & robots, const std::string & main)
{
  std::pair<std::map<std::string, std::vector<double>>, std::map<std::string, sva::PTransformd>> out;
  auto & encoders = out.first;
  auto & bases = out.second;
  for(const auto & r : robots)
  {
    bool is_main = r.name() == main;
    auto r_encoders = get_initial_encoders(log, r, is_main);
    if(r_encoders) { encoders[r.name()] = *r_encoders; }
    auto posW = get_posW(log, r.name(), is_main, 0);
    if(posW) { bases[r.name()] = *posW; }
  }
  return out;
}

auto get_gc_configuration = [](const Ticker::Configuration & config)
{
  mc_control::MCGlobalController::GlobalConfiguration out(config.mc_rtc_configuration);
  if(config.replay_configuration.log.size())
  {
    auto it = std::find(out.global_plugins.begin(), out.global_plugins.end(), "Replay");
    if(it != out.global_plugins.end()) { out.global_plugins.erase(it); }
    out.global_plugins.insert(out.global_plugins.begin(), "Replay");
    auto replay_c = out.config.add("Replay");
    replay_c.add("with-inputs", config.replay_configuration.with_inputs);
    replay_c.add("with-gui-inputs",
                 config.replay_configuration.with_gui_inputs && !config.replay_configuration.with_outputs);
    replay_c.add("with-outputs", config.replay_configuration.with_outputs);
    replay_c.add("with-datastore-config", config.replay_configuration.with_datastore_config);
    if(config.replay_configuration.with_outputs)
    {
      out.enabled_controllers = {"Passthrough"};
      out.initial_controller = "Passthrough";
    }
  }
  return out;
};

Ticker::Ticker(const Configuration & config) : config_(config), gc_(get_gc_configuration(config))
{
  auto & replay_c = config_.replay_configuration;
  if(!replay_c.log.empty())
  {
    replay_c.log = bfs::system_complete(replay_c.log).string();
    std::map<std::string, std::string> log_to_datastore;
    if(!replay_c.with_datastore_config.empty())
    {
      log_to_datastore =
          mc_rtc::Configuration(replay_c.with_datastore_config).operator std::map<std::string, std::string>();
    }
    log_ = std::make_shared<mc_rtc::log::FlatLog>(replay_c.log);
    gc_.controller().datastore().make<decltype(log_)>("Replay::Log", log_);
  }
  simulate_sensors();
  if(log_)
  {
    // Check that the replay timestep matches the configured one
    if(log_->size() > 1)
    {
      double dt = log_->get("t", 1, 0.0) - log_->get("t", 0, 0.0);
      if(std::fabs(dt - gc_.timestep()) > 1e-6)
      {
        mc_rtc::log::error_and_throw("Controller timestep ({}) is different from replay timestep ({})", gc_.timestep(),
                                     dt);
      }
    }
    // Do the initialization
    auto [encoders, attitudes] = get_initial_state(*log_, gc_.robots(), gc_.controller().robot().name());
    gc_.init(encoders, attitudes);
  }
  else { gc_.init(); }
  gc_.running = true;
  setup_gui();
}

void Ticker::reset()
{
  config_.step_by_step = false;
  do_reset_ = true;
}

bool Ticker::step()
{
  if(do_reset_)
  {
    do_reset_ = false;
    iters_ = 0;
    replay_done_ = false;
    simulate_sensors();
    if(log_)
    {
      auto [encoders, attitudes] = get_initial_state(*log_, gc_.robots(), gc_.controller().robot().name());
      gc_.reset(encoders, attitudes);
    }
    else { gc_.reset(); }
    gc_.running = true;
    setup_gui();
  }
  simulate_sensors();
  bool r = gc_.run();
  iters_++;
  return r;
}

void Ticker::run()
{
  using clock = std::conditional_t<std::chrono::high_resolution_clock::is_steady, std::chrono::high_resolution_clock,
                                   std::chrono::steady_clock>;
  using duration_us = std::chrono::duration<double, std::micro>;

  replay_done_ = false;
  running_ = true;
  iters_ = 0;
  double sim_elapsed_t = 0.0;
  double real_elapsed_t = 0.0;

  double target_ratio = config_.sync_ratio;
  gc_.server().update_rate(gc_.timestep() / target_ratio, gc_.configuration().gui_server_configuration.timestep);

  auto start_ticker = clock::now();

  auto reset_sync = [&]()
  {
    real_elapsed_t = 0.0;
    sim_elapsed_t = 0.0;
    start_ticker = clock::now();
  };

  bool was_no_sync = config_.no_sync;

  while(running_ && elapsed_time() < config_.run_for)
  {
    if(config_.sync_ratio != target_ratio || do_reset_)
    {
      target_ratio = config_.sync_ratio;
      gc_.server().update_rate(gc_.timestep() / target_ratio, gc_.configuration().gui_server_configuration.timestep);
      reset_sync();
    }
    if(((config_.step_by_step && rem_steps_ > 0) || !config_.step_by_step) && gc_.running)
    {
      rem_steps_--;
      step();
      sim_elapsed_t += gc_.timestep();
      auto end_step = clock::now();
      real_elapsed_t = duration_us{end_step - start_ticker}.count() / 1e6;
      sim_real_ratio_ = sim_elapsed_t / real_elapsed_t;
      if(!config_.no_sync)
      {
        if(was_no_sync) { reset_sync(); }
        // sim_elased_t / (real_elapsed_t + delay) = target_ratio
        // delay = sim / ratio - real;
        double delay = sim_elapsed_t / target_ratio - real_elapsed_t;
        std::this_thread::sleep_for(duration_us(1e6 * delay));
      }
    }
    else
    {
      // Only update the GUI
      bool was_running = gc_.running;
      gc_.running = false;
      gc_.run();
      gc_.running = was_running;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      reset_sync();
    }
    was_no_sync = config_.no_sync;
    if(log_ && config_.replay_configuration.stop_after_log && iters_ == log_->size() && !replay_done_)
    {
      replay_done_ = true;
      config_.step_by_step = true;
      if(config_.replay_configuration.exit_after_log)
      {
        running_ = false;
        mc_rtc::log::success("Log replay finished, exit replay now");
      }
      else { mc_rtc::log::success("Log replay finished, pausing replay now"); }
    }
  }
}

void Ticker::setup_gui()
{
  auto & gui = *gc_.controller().gui();
  gui.removeElements(this);
  gui.addElement(
      this, {"Ticker"}, mc_rtc::gui::Button("Stop", [this]() { running_ = false; }),
      mc_rtc::gui::Button("Reset", [this]() { reset(); }),
      mc_rtc::gui::Checkbox(
          "Synchronize", [this]() { return !config_.no_sync; }, [this]() { config_.no_sync = !config_.no_sync; }));
  gui.addElement(this, {"Ticker"}, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Label("Ratio", [this]() { return fmt::format("{:0.2f}", sim_real_ratio_); }),
                 mc_rtc::gui::NumberInput(
                     "Target ratio", [this]() { return config_.sync_ratio; },
                     [this](double r)
                     {
                       if(r <= 0.0) { r = 1.0 / 1024.0; }
                       config_.sync_ratio = r;
                     }),
                 mc_rtc::gui::Button("x2", [this]() { config_.sync_ratio *= 2.0; }),
                 mc_rtc::gui::Button("/2", [this]() { config_.sync_ratio /= 2.0; }));
  gui.addElement(this, {"Ticker"}, mc_rtc::gui::Checkbox("Step by step", config_.step_by_step));
  auto dt = gc_.timestep();
  auto text = [&](size_t n)
  {
    size_t n_ms = static_cast<size_t>(std::ceil(static_cast<double>(n * 1000) * dt));
    return fmt::format("+{}ms", n_ms);
  };
  auto button = [&, this](size_t n)
  {
    return mc_rtc::gui::Button(text(n),
                               [this, n]()
                               {
                                 if(rem_steps_ < 0) { rem_steps_ = 0; }
                                 rem_steps_ += n;
                               });
  };
  gui.addElement(this, {"Ticker"}, mc_rtc::gui::ElementsStacking::Horizontal, button(1), button(5), button(10),
                 button(50), button(100));
  if(log_)
  {
    gui.addElement(this, {"Ticker"}, mc_rtc::gui::Label("Replay", config_.replay_configuration.log),
                   mc_rtc::gui::NumberSlider(
                       "Replay time", [this]() { return elapsed_time(); }, [this](double t) { set_time(t); }, 0.0,
                       static_cast<double>(log_->size()) * dt));
  }
}

void Ticker::simulate_sensors()
{
  if(log_ && iters_ < log_->size() && config_.replay_configuration.with_inputs)
  {
    // Sensors are handled by the Replay plugin
  }
  else
  {
    for(const auto & r : gc_.controller().outputRobots())
    {
      gc_.setEncoderValues(r.name(), get_encoders(r));
      gc_.setEncoderVelocities(r.name(), get_encoders_velocities(r));
      if(r.hasBodySensor("FloatingBase"))
      {
        gc_.setSensorPositions(r.name(), {{"FloatingBase", r.posW().translation()}});
        gc_.setSensorOrientations(r.name(), {{"FloatingBase", Eigen::Quaterniond{r.posW().rotation()}}});
      }
    }
  }
}

void Ticker::set_time(double t)
{
  if(log_ && config_.replay_configuration.with_outputs)
  {
    size_t iter = static_cast<size_t>(std::floor(t / gc_.timestep()));
    iters_ = std::max<size_t>(std::min<size_t>(iter, log_->size() - 1), 0);
    gc_.controller().datastore().call("Replay::iter", iters_);
  }
}

} // namespace mc_control
