/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "GenericLoader.h"

#define STRINGIFY(T) #T

namespace mc_solver
{

template<typename Derived, typename T>
GenericLoader<Derived, T>::Handle::Handle(const std::string & type) : type_(type)
{
}

template<typename Derived, typename T>
GenericLoader<Derived, T>::Handle::Handle(Handle && h) : type_("")
{
  std::swap(h.type_, type_);
}

template<typename Derived, typename T>
typename GenericLoader<Derived, T>::Handle & GenericLoader<Derived, T>::Handle::operator=(Handle && h)
{
  if(&h == this)
  {
    return *this;
  }
  std::swap(h.type_, type_);
  return *this;
}

template<typename Derived, typename T>
GenericLoader<Derived, T>::Handle::~Handle()
{
  GenericLoader<Derived, T>::unregister_load_function(type_);
}

template<typename Derived, typename T>
typename GenericLoader<Derived, T>::T_ptr GenericLoader<Derived, T>::load(mc_solver::QPSolver & solver,
                                                                          const std::string & file)
{
  return load(solver, mc_rtc::Configuration(file));
}

template<typename Derived, typename T>
typename GenericLoader<Derived, T>::T_ptr GenericLoader<Derived, T>::load(mc_solver::QPSolver & solver,
                                                                          const mc_rtc::Configuration & config)
{
  static auto & fns = get_fns();
  if(config.has("type"))
  {
    std::string type = config("type");
    if(fns.count(type))
    {
      return fns[type](solver, config);
    }
    mc_rtc::log::error_and_throw<std::runtime_error>("GenericLoader cannot handle object type \"{}\"", type);
  }
  mc_rtc::log::error_and_throw<std::runtime_error>("Attempted to load an object without a type property");
}

template<typename Derived, typename T>
template<typename U, typename std::enable_if<(!std::is_same<U, T>::value) && std::is_base_of<T, U>::value, int>::type>
std::shared_ptr<U> GenericLoader<Derived, T>::load(mc_solver::QPSolver & solver, const std::string & file)
{
  return cast<U>(load(solver, file));
}

template<typename Derived, typename T>
template<typename U, typename std::enable_if<(!std::is_same<U, T>::value) && std::is_base_of<T, U>::value, int>::type>
std::shared_ptr<U> GenericLoader<Derived, T>::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  return cast<U>(load(solver, config));
}

template<typename Derived, typename T>
std::map<std::string, typename GenericLoader<Derived, T>::load_fun> & GenericLoader<Derived, T>::get_fns()
{
  return Derived::storage();
}

template<typename Derived, typename T>
template<typename U>
std::shared_ptr<U> GenericLoader<Derived, T>::cast(const GenericLoader<Derived, T>::T_ptr & mt)
{
  auto ret = std::dynamic_pointer_cast<U>(mt);
  if(!ret)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "The object stored in the JSON object is not of the requested type");
  }
  return ret;
}

template<typename Derived, typename T>
typename GenericLoader<Derived, T>::Handle GenericLoader<Derived, T>::register_load_function(const std::string & type,
                                                                                             load_fun fn)
{
  static auto & fns = get_fns();
  if(fns.count(type) == 0)
  {
    fns[type] = fn;
    return Handle(type);
  }
  mc_rtc::log::error_and_throw<std::runtime_error>("{} is already handled by another loading function", type);
}

template<typename Derived, typename T>
void GenericLoader<Derived, T>::unregister_load_function(const std::string & type)
{
  static auto & fns = get_fns();
  if(fns.count(type))
  {
    fns.erase(type);
  }
}

} // namespace mc_solver
