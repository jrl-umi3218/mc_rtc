#pragma once

#include <functional>

#include <Eigen/Core>
#include <SpaceVecAlg/SpaceVecAlg>

#include <mc_rtc/Configuration.h>
#include <mc_rbdyn/configuration_io.h>

template<typename T>
using function = std::function<T()>;

template<typename retT, typename T, typename U>
std::function<retT()> make_log_callback(T fn, U arg)
{
  return [fn, arg]{ return fn(arg); };
};

#define MAKE_LOG_HELPER(NAME, TYPE)\
  template<typename T, typename U>\
  std::function<TYPE()> NAME(T fn, U arg)\
  {\
    return make_log_callback<TYPE>(fn ,arg);\
  }
MAKE_LOG_HELPER(make_v3d_log_callback, Eigen::Vector3d)
MAKE_LOG_HELPER(make_double_log_callback, double)
MAKE_LOG_HELPER(make_doublev_log_callback, std::vector<double>)
MAKE_LOG_HELPER(make_quat_log_callback, Eigen::Quaterniond)
MAKE_LOG_HELPER(make_pt_log_callback, sva::PTransformd)
MAKE_LOG_HELPER(make_fv_log_callback, sva::ForceVecd)
MAKE_LOG_HELPER(make_string_log_callback, std::string)

template<typename T>
T get_config_as(mc_rtc::Configuration & config)
{
  T ret = config;
  return ret;
}

template<typename T>
T get_config_as(mc_rtc::Configuration & config, const T & def)
{
  try
  {
    return get_config_as<T>(config);
  }
  catch(const mc_rtc::Configuration::Exception&)
  {
    return def;
  }
}

template<typename T>
mc_rtc::Configuration get_as_config(const T & v)
{
  mc_rtc::Configuration conf;
  conf.add("v", v);
  return conf("v");
}

mc_rtc::Configuration ConfigurationFromData(const std::string & data)
{
  return mc_rtc::Configuration::fromData(data);
}
