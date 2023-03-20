#include <mc_control/Ticker.h>

namespace mc_control
{

/** Get the encoders of a robot from the given log at the given time */
std::vector<double> get_encoders(const mc_rtc::log::FlatLog & log, const std::string & robot, bool is_main, double t)
{
  // FIXME Implement
  return {};
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
    for(const auto & qi : robot.mbc().q[static_cast<size_t>(mbcIdx)])
    {
      q.push_back(qi);
    }
  }
  return q;
}

/** Get the floating base position from the given log at the given time */
sva::PTransformd get_posW(const mc_rtc::log::FlatLog & log, const std::string & robot, bool is_main, double t)
{
  // FIXME Implement
  return sva::PTransformd::Identity();
}

/** Get the initial state of all robots from the given log */
auto get_initial_state(const mc_rtc::log::FlatLog & log,
                       const std::vector<std::string> & robots,
                       const std::string & main)
{
  std::pair<std::map<std::string, std::vector<double>>, std::map<std::string, sva::PTransformd>> out;
  auto & encoders = out.first;
  auto & bases = out.second;
  for(const auto & r : robots)
  {
    encoders[r] = get_encoders(log, r, r == main, 0.0);
    bases[r] = get_posW(log, r, r == main, 0.0);
  }
  return out;
}

/** Get the names of the robot currently in the controller */
std::vector<std::string> get_robots(const mc_control::MCGlobalController & gc)
{
  std::vector<std::string> out;
  for(const auto & r : gc.controller().robots())
  {
    out.push_back(r.name());
  }
  return out;
}

Ticker::Ticker(const Configuration & config) : config_(config), gc_(config_.mc_rtc_configuration)
{
  const auto & replay_c = config_.replay_configuration;
  if(!replay_c.log.empty())
  {
    std::map<std::string, std::string> log_to_datastore;
    if(!replay_c.with_datastore_config.empty())
    {
      log_to_datastore =
          mc_rtc::Configuration(replay_c.with_datastore_config).operator std::map<std::string, std::string>();
    }
    replay_ = ReplayData{mc_rtc::log::FlatLog(replay_c.log), log_to_datastore};
  }
  simulate_sensors();
  if(replay_)
  {
    // Check that the replay timestep matches the configured one
    if(replay_->log.size() > 1)
    {
      double dt = replay_->log.get("t", 1, 0.0) - replay_->log.get("t", 0, 0.0);
      if(dt != gc_.timestep())
      {
        mc_rtc::log::error_and_throw("Controller timestep ({}) is different from replay timestep ({})", gc_.timestep(),
                                     dt);
      }
    }
    // Do the initialization
    auto [encoders, attitudes] = get_initial_state(replay_->log, get_robots(gc_), gc_.controller().robot().name());
    gc_.init(encoders, attitudes);
  }
  else
  {
    gc_.init();
  }
  gc_.running = true;
  setup_gui();
}

void Ticker::reset()
{
  do_reset_ = true;
}

bool Ticker::step()
{
  if(do_reset_)
  {
    do_reset_ = false;
    elapsed_t_ = 0.0;
    simulate_sensors();
    if(replay_)
    {
      auto [encoders, attitudes] = get_initial_state(replay_->log, get_robots(gc_), gc_.controller().robot().name());
      gc_.reset(encoders, attitudes);
    }
    else
    {
      gc_.reset();
    }
    gc_.running = true;
    setup_gui();
  }
  simulate_sensors();
  bool r = gc_.run();
  if(replay_)
  {
    // FIXME For replay output to work we need to:
    // - Insert a plugin at the start of the global plugin list (overwrite the output robot for every plugins)
    // - Publish GUI robots ourselves
  }
  iters_++;
  elapsed_t_ += gc_.timestep();
  return r;
}

void Ticker::run()
{
  using clock = std::conditional_t<std::chrono::high_resolution_clock::is_steady, std::chrono::high_resolution_clock,
                                   std::chrono::steady_clock>;
  using duration_us = std::chrono::duration<double, std::micro>;
  auto dt = duration_us(1e6 * gc_.timestep());

  running_ = true;
  iters_ = 0;
  elapsed_t_ = 0.0;

  while(running_ && elapsed_t_ < config_.run_for)
  {
    auto now = clock::now();
    if(config_.step_by_step)
    {
      if(rem_steps_ > 0)
      {
        rem_steps_--;
        step();
      }
      else
      {
        // Only update the GUI
        gc_.running = false;
        gc_.run();
        gc_.running = true;
      }
    }
    else
    {
      step();
    }
    if(!config_.no_sync)
    {
      // FIXME Implement compensation
      std::this_thread::sleep_until(now + dt);
    }
    if(replay_ && config_.replay_configuration.stop_after_log && iters_ >= replay_->log.size())
    {
      mc_rtc::log::success("Finihed replaying log");
      running_ = false;
    }
  }
}

void Ticker::setup_gui()
{
  auto & gui = *gc_.controller().gui();
  gui.removeElements(this);
  gui.addElement(this, {"Ticker"}, mc_rtc::gui::Button("Stop", [this]() { running_ = false; }),
                 mc_rtc::gui::Button("Reset", [this]() { reset(); }),
                 mc_rtc::gui::Checkbox(
                     "Sync with real-time", [this]() { return !config_.no_sync; },
                     [this]() { config_.no_sync = !config_.no_sync; }),
                 mc_rtc::gui::Checkbox("Step by step", config_.step_by_step));
  auto dt = gc_.timestep();
  auto text = [&](size_t n) {
    size_t n_ms = static_cast<size_t>(std::ceil(static_cast<double>(n * 1000) * dt));
    return fmt::format("+{}ms", n_ms);
  };
  auto button = [&, this](size_t n) { return mc_rtc::gui::Button(text(n), [this, n]() { rem_steps_ += n; }); };
  gui.addElement(this, {"Ticker"}, mc_rtc::gui::ElementsStacking::Horizontal, button(1), button(5), button(10),
                 button(50), button(100));
}

void Ticker::simulate_sensors()
{
  if(replay_ && iters_ < replay_->log.size())
  {
    // FIXME Implement sensor playback
  }
  else
  {
    for(const auto & r : gc_.controller().robots())
    {
      gc_.setEncoderValues(r.name(), get_encoders(r));
    }
  }
}

} // namespace mc_control
