/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/loader.h>

#include <sstream>

namespace mc_rtc
{

template<typename SymT>
SymT LTDLHandle::get_symbol(const std::string & name)
{
#ifndef MC_RTC_BUILD_STATIC
  if(!open())
  {
    return nullptr;
  }
  std::unique_lock<std::mutex> lock(LTDLMutex::MTX);
  SymT ret = (SymT)(lt_dlsym(handle_, name.c_str()));
  if(ret == nullptr)
  {
    const char * error = lt_dlerror();
    if(verbose_)
    {
      mc_rtc::log::warning("Could not get symbol {} in library {}\n{}", name, path_, error);
    }
  }
  return ret;
#else
  return nullptr;
#endif
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
                              bool verbose,
                              Loader::callback_t cb)
: class_name(class_name), verbose(verbose)
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
  return handles_.count(object) != 0 || callbacks_.has(object);
}

template<typename T>
std::vector<std::string> ObjectLoader<T>::objects() const
{
  std::vector<std::string> res = callbacks_.keys();
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
  callbacks_.clear();
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
    mc_rtc::log::error_and_throw<LoaderException>("Requested creation of object named {} which has not been loaded",
                                                  name);
  }
  if(handles_.count(name))
  {
    return create_from_handles(name, std::forward<Args>(args)...);
  }
  else
  {
    return create_from_callbacks(name, std::forward<Args>(args)...);
  }
}

template<typename T>
template<typename... Args>
T * ObjectLoader<T>::create_from_handles(const std::string & name, Args... args)
{
  unsigned int args_passed = 1 + sizeof...(Args);
  unsigned int args_required = args_passed;
  auto create_args_required = handles_[name]->template get_symbol<unsigned int (*)()>("create_args_required");
  if(create_args_required != nullptr)
  {
    args_required = create_args_required();
  }
  if(args_passed != args_required)
  {
    mc_rtc::log::error_and_throw<LoaderException>("{} arguments passed to create function of {} which excepts {}",
                                                  args_passed, name, args_required);
  }
  auto create_fn =
      handles_[name]->template get_symbol<T * (*)(const std::string &, const typename std::decay<Args>::type &...)>(
          "create");
  if(create_fn == nullptr)
  {
    mc_rtc::log::error_and_throw<LoaderException>("Failed to resolve create symbol in {}", handles_[name]->path());
  }
  T * ptr = create_fn(name, args...);
  if(ptr == nullptr)
  {
    mc_rtc::log::error_and_throw<LoaderException>("Call to create for object {} failed", name);
  }
  if(!deleters_.count(name))
  {
    auto delete_fn = handles_[name]->template get_symbol<void (*)(T *)>("destroy");
    if(delete_fn == nullptr)
    {
      mc_rtc::log::error_and_throw<LoaderException>("Symbol destroy not found in {}", handles_[name]->path());
    }
    deleters_[name] = ObjectDeleter(delete_fn);
  }
  return ptr;
}

template<typename T>
template<typename... Args>
T * ObjectLoader<T>::create_from_callbacks(const std::string & name, Args... args)
{
  return callbacks_.call<T *, const typename std::decay<Args>::type &...>(name, std::forward<Args>(args)...);
}

template<typename T>
template<typename RetT, typename... Args>
void ObjectLoader<T>::register_object(const std::string & name, std::function<RetT *(const Args &...)> callback)
{
  if(has_object(name))
  {
    throw LoaderException(fmt::format("{} is already registered with this loader", name));
  }
  static_assert(std::is_base_of<T, RetT>::value,
                "This object cannot be registered as it does not derive from the loader base-class");
  callbacks_.make_call(name, [callback](const Args &... args) -> T * { return callback(args...); });
  deleters_[name] = ObjectDeleter([](T * ptr) { delete static_cast<RetT *>(ptr); });
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
