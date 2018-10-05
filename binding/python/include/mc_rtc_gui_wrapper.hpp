#pragma once

#include <functional>

#include <Eigen/Core>
#include <SpaceVecAlg/SpaceVecAlg>

#include <mc_rtc/Configuration.h>
#include <mc_rbdyn/configuration_io.h>

using get_fn = std::function<mc_rtc::Configuration()>;
using set_fn = std::function<void(const mc_rtc::Configuration&)>;
using void_cb = std::function<void()>;

template<typename T, typename U>
get_fn make_getter(T fn, U arg)
{
  return [fn, arg]{ return fn(arg); };
}

template<typename T, typename Cb, typename t>
set_fn make_setter(T fn, Cb cb, t type)
{
  return [fn, cb, type](const mc_rtc::Configuration & data) mutable
  {
    return fn(cb, type, const_cast<mc_rtc::Configuration&>(data));
  };
}

template<typename T, typename Cb>
void_cb make_void_cb(T fn, Cb cb)
{
  return [fn,cb]() mutable { return fn(cb); };
}
