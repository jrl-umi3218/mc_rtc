/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Replay.h"

#include <mc_control/GlobalPluginMacros.h>

#include <mc_control/Ticker.h>

namespace mc_plugin
{

namespace
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

} // namespace

void Replay::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config)
{
  if(config.empty() && gc.configuration().config.has("Replay"))
  {
    return init(gc, gc.configuration().config("Replay"));
  }
  const auto & ds = gc.controller().datastore();
  if(ds.has("Replay::Log"))
  {
    log_ = ds.get<decltype(log_)>("Replay::Log");
  }
  else
  {
    if(!config.has("log"))
    {
      mc_rtc::log::error_and_throw(
          "[Replay] No log specified in the plugin configuration and no log available in the datastore at Replay::Log");
    }
    log_ = std::make_shared<mc_rtc::log::FlatLog>(config("log").operator std::string());
  }
  std::string config_str;
  auto do_config = [&](const char * key, bool & check, std::string_view msg) {
    config(key, check);
    if(check)
    {
      if(config_str.size())
      {
        config_str += ", ";
      }
      config_str += msg;
    }
  };
  do_config("with-inputs", with_inputs_, "replay sensor inputs");
  do_config("with-gui-inputs", with_gui_inputs_, "replay GUI inputs");
  do_config("with-outputs", with_outputs_, "replay controller output");
  if(config_str.size())
  {
    mc_rtc::log::info("[Replay] Will {}", config_str);
  }
  else
  {
    mc_rtc::log::warning("[Replay] Configured to do nothing?");
  }
  std::string with_datastore_config = config("with-datastore-config", std::string(""));
  if(!with_datastore_config.empty())
  {
    mc_rtc::log::info("[Replay] Loading log to datastore configuration from {}", with_datastore_config);
    log_to_datastore_ = mc_rtc::Configuration(with_datastore_config).operator std::map<std::string, std::string>();
  }
  ctl_name_ = gc.controller().name_;
  reset(gc);
}

void Replay::reset(mc_control::MCGlobalController & gc)
{
  iters_ = 0;
  if(gc.controller().name_ != ctl_name_)
  {
    mc_rtc::log::warning(
        "[Replay] Reset with a different controller than the initial one, jumping to the end of the log");
    iters_ = log_->size();
  }
  // Run once to fill the initial sensors
  before(gc);
  iters_ = 0;
}

void Replay::before(mc_control::MCGlobalController & gc)
{
  if(iters_ >= log_->size())
  {
    return;
  }
  const auto & log = *log_;
  if(with_inputs_)
  {
    for(const auto & r : gc.controller().robots())
    {
      bool is_main = r.name() == gc.controller().robot().name();
      // Restore joint level readings
      if(r.refJointOrder().size())
      {
        gc.setEncoderValues(r.name(), get(log, "qIn", r.name(), is_main, iters_));
        gc.setEncoderVelocities(r.name(), get(log, "alphaIn", r.name(), is_main, iters_));
        gc.setJointTorques(r.name(), get(log, "tauIn", r.name(), is_main, iters_));
      }
      // Restore force sensor readings
      std::map<std::string, sva::ForceVecd> wrenches;
      for(const auto & fs : r.forceSensors())
      {
        wrenches[fs.name()] = get(log, fs.name(), r.name(), is_main, iters_, sva::ForceVecd::Zero());
      }
      gc.setWrenches(r.name(), wrenches);
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
      gc.setSensorPositions(r.name(), poses);
      gc.setSensorOrientations(r.name(), oris);
      gc.setSensorLinearVelocities(r.name(), linearVels);
      gc.setSensorAngularVelocities(r.name(), angularVels);
      gc.setSensorLinearAccelerations(r.name(), linearAccels);
      gc.setSensorAngularAccelerations(r.name(), angularAccels);
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
      gc.setJointMotorTemperatures(r.name(), motorTemps);
      gc.setJointDriverTemperatures(r.name(), driverTemps);
      gc.setJointMotorCurrents(r.name(), motorCurrents);
    }
  }
  if(with_gui_inputs_)
  {
    gc.server().push_requests(log.guiEvents()[iters_]);
  }
  // FIXME Handle log_to_datastore
}

void Replay::after(mc_control::MCGlobalController & gc)
{
  if(with_outputs_)
  {
    // FIXME implement
    (void)gc;
  }
  iters_++;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("Replay", mc_plugin::Replay)
