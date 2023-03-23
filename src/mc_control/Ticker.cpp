#include <mc_control/Ticker.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_control
{

template<typename StrT>
std::string log_entry(const StrT & entry, const std::string & robot, bool is_main)
{
  if(is_main)
  {
    return entry;
  }
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
    for(const auto & qi : robot.mbc().q[static_cast<size_t>(mbcIdx)])
    {
      q.push_back(qi);
    }
  }
  return q;
}

/** Get the floating base position from the given log at the given time */
std::optional<sva::PTransformd> get_posW(const mc_rtc::log::FlatLog & log,
                                         const std::string & robot,
                                         bool is_main,
                                         size_t idx)
{
  auto entry = log_entry("ff", robot, is_main);
  if(log.meta())
  {
    auto it = log.meta()->init.find(robot);
    if(it != log.meta()->init.end())
    {
      return it->second;
    }
  }
  if(!log.has(entry))
  {
    return std::nullopt;
  }
  return log.get(entry, idx, sva::PTransformd::Identity());
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
    bool is_main = r == main;
    if(log.has(log_entry("qIn", r, is_main)))
    {
      encoders[r] = get_encoders(log, r, is_main, 0);
    }
    auto posW = get_posW(log, r, is_main, 0);
    if(posW)
    {
      bases[r] = *posW;
    }
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
  gc_.server().update_rate(gc_.timestep() / target_ratio, gc_.configuration().gui_timestep);

  auto start_ticker = clock::now();

  auto reset_sync = [&]() {
    real_elapsed_t = 0.0;
    sim_elapsed_t = 0.0;
    start_ticker = clock::now();
  };

  while(running_ && elapsed_time() < config_.run_for)
  {
    if(config_.sync_ratio != target_ratio || do_reset_)
    {
      target_ratio = config_.sync_ratio;
      gc_.server().update_rate(gc_.timestep() / target_ratio, gc_.configuration().gui_timestep);
      reset_sync();
    }
    if((config_.step_by_step && rem_steps_ > 0) || !config_.step_by_step)
    {
      rem_steps_--;
      step();
      sim_elapsed_t += gc_.timestep();
      auto end_step = clock::now();
      real_elapsed_t = duration_us{end_step - start_ticker}.count() / 1e6;
      sim_real_ratio_ = sim_elapsed_t / real_elapsed_t;
      if(!config_.no_sync)
      {
        // sim_elased_t / (real_elapsed_t + delay) = target_ratio
        // delay = sim / ratio - real;
        double delay = sim_elapsed_t / target_ratio - real_elapsed_t;
        std::this_thread::sleep_for(duration_us(1e6 * delay));
      }
    }
    else
    {
      // Only update the GUI
      gc_.running = false;
      gc_.run();
      gc_.running = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      reset_sync();
    }
    if(replay_ && config_.replay_configuration.stop_after_log && iters_ == replay_->log.size() && !replay_done_)
    {
      replay_done_ = true;
      config_.step_by_step = true;
      if(config_.replay_configuration.exit_after_log)
      {
        running_ = false;
        mc_rtc::log::success("Log replay finished, exit replay now");
      }
      else
      {
        mc_rtc::log::success("Log replay finished, pausing replay now");
      }
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
                     [this](double r) {
                       if(r <= 0.0)
                       {
                         r = 1.0 / 1024.0;
                       }
                       config_.sync_ratio = r;
                     }),
                 mc_rtc::gui::Button("x2", [this]() { config_.sync_ratio *= 2.0; }),
                 mc_rtc::gui::Button("/2", [this]() { config_.sync_ratio /= 2.0; }));
  gui.addElement(this, {"Ticker"}, mc_rtc::gui::Checkbox("Step by step", config_.step_by_step));
  auto dt = gc_.timestep();
  auto text = [&](size_t n) {
    size_t n_ms = static_cast<size_t>(std::ceil(static_cast<double>(n * 1000) * dt));
    return fmt::format("+{}ms", n_ms);
  };
  auto button = [&, this](size_t n) {
    return mc_rtc::gui::Button(text(n), [this, n]() {
      if(rem_steps_ < 0)
      {
        rem_steps_ = 0;
      }
      rem_steps_ += n;
    });
  };
  gui.addElement(this, {"Ticker"}, mc_rtc::gui::ElementsStacking::Horizontal, button(1), button(5), button(10),
                 button(50), button(100));
  if(replay_)
  {
    gui.addElement(this, {"Ticker"}, mc_rtc::gui::Label("Replay", config_.replay_configuration.log),
                   mc_rtc::gui::NumberSlider(
                       "Replay time", [this]() { return elapsed_time(); }, [](double) {}, 0.0,
                       static_cast<double>(replay_->log.size()) * dt));
  }
}

void Ticker::simulate_sensors()
{
  if(replay_ && iters_ < replay_->log.size())
  {
    const auto & log = replay_->log;
    if(config_.replay_configuration.with_inputs)
    {
      for(const auto & r : gc_.controller().robots())
      {
        bool is_main = r.name() == gc_.controller().robot().name();
        // Restore joint level readings
        if(r.refJointOrder().size())
        {
          gc_.setEncoderValues(r.name(), get_encoders(replay_->log, r.name(), is_main, iters_));
          gc_.setEncoderVelocities(r.name(), get(log, "alphaIn", r.name(), is_main, iters_));
          gc_.setJointTorques(r.name(), get(log, "tauIn", r.name(), is_main, iters_));
        }
        // Restore force sensor readings
        std::map<std::string, sva::ForceVecd> wrenches;
        for(const auto & fs : r.forceSensors())
        {
          wrenches[fs.name()] = get(log, fs.name(), r.name(), is_main, iters_, sva::ForceVecd::Zero());
        }
        gc_.setWrenches(r.name(), wrenches);
        // Restore body sensor readings
        std::map<std::string, Eigen::Vector3d> poses;
        mc_control::MCGlobalController::QuaternionMap oris;
        std::map<std::string, Eigen::Vector3d> linearVels;
        std::map<std::string, Eigen::Vector3d> angularVels;
        std::map<std::string, Eigen::Vector3d> linearAccels;
        std::map<std::string, Eigen::Vector3d> angularAccels;
        static auto def_quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
        static Eigen::Vector3d def_vec = Eigen::Vector3d::Zero();
        for(const auto & bs : r.bodySensors())
        {
          poses[bs.name()] = get(log, bs.name() + "_position", r.name(), is_main, iters_, def_vec);
          oris[bs.name()] = get(log, bs.name() + "_orientation", r.name(), is_main, iters_, def_quat);
          linearVels[bs.name()] = get(log, bs.name() + "_linearVelocity", r.name(), is_main, iters_, def_vec);
          angularVels[bs.name()] = get(log, bs.name() + "_angularVelocity", r.name(), is_main, iters_, def_vec);
          linearAccels[bs.name()] = get(log, bs.name() + "_linearAcceleration", r.name(), is_main, iters_, def_vec);
          angularAccels[bs.name()] = get(log, bs.name() + "_angularAcceleration", r.name(), is_main, iters_, def_vec);
        }
        gc_.setSensorPositions(r.name(), poses);
        gc_.setSensorOrientations(r.name(), oris);
        gc_.setSensorLinearVelocities(r.name(), linearVels);
        gc_.setSensorAngularVelocities(r.name(), angularVels);
        gc_.setSensorLinearAccelerations(r.name(), linearAccels);
        gc_.setSensorAngularAccelerations(r.name(), angularAccels);
        // Restore joint sensor readings
        std::map<std::string, double> motorTemps;
        std::map<std::string, double> driverTemps;
        std::map<std::string, double> motorCurrents;
        for(const auto & js : r.jointSensors())
        {
          motorTemps[js.joint()] =
              get(log, "JointSensor_" + js.joint() + "_motorTemperature", r.name(), is_main, iters_, 0.0);
          driverTemps[js.joint()] =
              get(log, "JointSensor_" + js.joint() + "_driverTemperature", r.name(), is_main, iters_, 0.0);
          motorCurrents[js.joint()] =
              get(log, "JointSensor_" + js.joint() + "_motorCurrent", r.name(), is_main, iters_, 0.0);
        }
        gc_.setJointMotorTemperatures(r.name(), motorTemps);
        gc_.setJointDriverTemperatures(r.name(), driverTemps);
        gc_.setJointMotorCurrents(r.name(), motorCurrents);
      }
    }
    if(config_.replay_configuration.with_gui_inputs)
    {
      gc_.server().push_requests(log.guiEvents()[iters_]);
    }
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
