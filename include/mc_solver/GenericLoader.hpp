#pragma once

#include "GenericLoader.h"

#define STRINGIFY(T) #T

namespace mc_solver
{

template<typename T>
typename GenericLoader<T>::T_ptr GenericLoader<T>::load(
                                 mc_solver::QPSolver & solver,
                                 const std::string & file)
{
  return load(solver, mc_rtc::Configuration(file));
}

template<typename T>
typename GenericLoader<T>::T_ptr GenericLoader<T>::load(
                                 mc_solver::QPSolver & solver,
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
    LOG_ERROR_AND_THROW(std::runtime_error, "GenericLoader cannot handle object type " << type)
  }
  LOG_ERROR_AND_THROW(std::runtime_error, "Attempted to load an object without a type property")
}

template<typename T>
template<typename U,
  typename std::enable_if<(!std::is_same<U, T>::value) &&
                          std::is_base_of<T, U>::value, int>::type>
std::shared_ptr<U> GenericLoader<T>::load(mc_solver::QPSolver & solver,
                                          const std::string & file)
{
  return cast<U>(load(solver, file));
}

template<typename T>
template<typename U,
  typename std::enable_if<(!std::is_same<U, T>::value) &&
                          std::is_base_of<T, U>::value, int>::type>
std::shared_ptr<U> GenericLoader<T>::load(mc_solver::QPSolver & solver,
                                          const mc_rtc::Configuration & config)
{
  return cast<U>(load(solver, config));
}

template<typename T>
std::unique_ptr<std::map<std::string, typename GenericLoader<T>::load_fun>> GenericLoader<T>::fns_ptr;

template<typename T>
std::map<std::string, typename GenericLoader<T>::load_fun> & GenericLoader<T>::get_fns()
{
  if(!fns_ptr)
  {
    fns_ptr = std::unique_ptr<std::map<std::string, load_fun>>(new std::map<std::string, load_fun>());
  }
  return *fns_ptr;
}

template<typename T>
template<typename U>
std::shared_ptr<U> GenericLoader<T>::cast(const GenericLoader<T>::T_ptr & mt)
{
  auto ret = std::dynamic_pointer_cast<U>(mt);
  if(!ret)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "The object stored in the JSON object is not of the requested type")
  }
  return ret;
}

template<typename T>
bool GenericLoader<T>::register_load_function(const std::string & type,
                                              load_fun fn)
{
  static auto & fns = get_fns();
  if(fns.count(type) == 0)
  {
    fns[type] = fn;
    return true;
  }
  LOG_ERROR_AND_THROW(std::runtime_error, type << " is already handled by another loading function")
  return false;
}

}
