/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/loader.h>
#include <mc_rtc/loader_sandbox.h>

#include <sstream>

namespace mc_rtc
{

template<typename SymT>
SymT LTDLHandle::get_symbol(const std::string & name)
{
  if(!open())
  {
    return nullptr;
  }
  SymT ret = (SymT)(lt_dlsym(handle_, name.c_str()));
  if(ret == nullptr)
  {
    const char * error = lt_dlerror();
    if(verbose_)
    {
      LOG_WARNING("Could not get symbol " << name << " in library " << path_ << "\n" << error)
    }
  }
  return ret;
}

template<typename T>
ObjectLoader<T>::ObjectDeleter::ObjectDeleter(void (*fn)(T *)) : delete_fn_(fn)
{
}

template<typename T>
void ObjectLoader<T>::ObjectDeleter::operator()(T * ptr)
{
  if(delete_fn_)
  {
    delete_fn_(ptr);
  }
}

template<typename T>
ObjectLoader<T>::ObjectLoader(const std::string & class_name,
                              const std::vector<std::string> & paths,
                              bool enable_sandbox,
                              bool verbose,
                              Loader::callback_t cb)
: class_name(class_name), enable_sandbox(enable_sandbox), verbose(verbose)
{
  Loader::init();
  load_libraries(paths, cb);
}

template<typename T>
ObjectLoader<T>::~ObjectLoader()
{
  Loader::close();
}

template<typename T>
bool ObjectLoader<T>::has_object(const std::string & object) const
{
  return handles_.count(object) != 0;
}

template<typename T>
std::vector<std::string> ObjectLoader<T>::objects() const
{
  std::vector<std::string> res;
  for(const auto & h : handles_)
  {
    res.push_back(h.first);
  }
  return res;
}

template<typename T>
void ObjectLoader<T>::load_libraries(const std::vector<std::string> & paths, Loader::callback_t cb)
{
  Loader::load_libraries(class_name, paths, handles_, verbose, cb);
}

template<typename T>
void ObjectLoader<T>::clear()
{
  handles_.clear();
}

template<typename T>
void ObjectLoader<T>::enable_sandboxing(bool enable_sandbox)
{
  this->enable_sandbox = enable_sandbox;
}

template<typename T>
void ObjectLoader<T>::set_verbosity(bool verbose)
{
  this->verbose = verbose;
}

template<typename T>
template<typename... Args>
T * ObjectLoader<T>::create(const std::string & name, Args... args)
{
  if(!has_object(name))
  {
    LOG_ERROR_AND_THROW(LoaderException, "Requested creation of object named " << name << " which has not been loaded")
  }
  unsigned int args_passed = 1 + sizeof...(Args);
  unsigned int args_required = args_passed;
  auto create_args_required = handles_[name]->template get_symbol<unsigned int (*)()>("create_args_required");
  if(create_args_required != nullptr)
  {
    args_required = create_args_required();
  }
  if(args_passed != args_required)
  {
    LOG_ERROR_AND_THROW(LoaderException, args_passed << " arguments passed to create function of " << name
                                                     << " which expects " << args_required)
  }
  auto create_fn =
      handles_[name]->template get_symbol<T * (*)(const std::string &, const typename std::decay<Args>::type &...)>(
          "create");
  if(create_fn == nullptr)
  {
    LOG_ERROR_AND_THROW(LoaderException, "Failed to resolve create symbol in " << handles_[name]->path() << "\n")
  }
  T * ptr = nullptr;
  if(enable_sandbox)
  {
    ptr = sandbox_function_call<T>(create_fn, name, args...);
  }
  else
  {
    ptr = no_sandbox_function_call<T>(create_fn, name, args...);
  }
  if(ptr == nullptr)
  {
    LOG_ERROR_AND_THROW(LoaderException, "Call to create for object " << name << " failed")
  }
  if(!deleters_.count(name))
  {
    auto delete_fn = handles_[name]->template get_symbol<void (*)(T *)>("destroy");
    if(delete_fn == nullptr)
    {
      LOG_ERROR_AND_THROW(LoaderException, "Symbol destroy not found in " << handles_[name]->path() << "\n")
    }
    deleters_[name] = ObjectDeleter(delete_fn);
  }
  return ptr;
}

template<typename T>
template<typename... Args>
std::shared_ptr<T> ObjectLoader<T>::create_object(const std::string & name, Args... args)
{
  T * ptr = create(name, std::forward<Args>(args)...);
  return std::shared_ptr<T>(ptr, deleters_[name]);
}

template<typename T>
template<typename... Args>
typename ObjectLoader<T>::unique_ptr ObjectLoader<T>::create_unique_object(const std::string & name, Args... args)
{
  T * ptr = create(name, std::forward<Args>(args)...);
  return unique_ptr(ptr, deleters_[name]);
}

} // namespace mc_rtc
