#pragma once

#include <mc_rtc/loader.h>
#include <mc_rtc/loader_sandbox.h>

namespace mc_rtc
{

template<typename T>
ObjectLoader<T>::ObjectDeleter::ObjectDeleter(void * sym)
{
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wpedantic"
  delete_fn_ = (void(*)(T*))(sym);
  #pragma GCC diagnostic pop
}

template<typename T>
void ObjectLoader<T>::ObjectDeleter::operator()(T * ptr)
{
  delete_fn_(ptr);
}

template<typename T>
ObjectLoader<T>::ObjectLoader(const std::vector<std::string> & paths)
{
  Loader::init();
  load_libraries(paths);
}

template<typename T>
ObjectLoader<T>::~ObjectLoader()
{
  for(const auto & h : handles_)
  {
    lt_dlclose(h.second);
  }
  Loader::close();
}

template<typename T>
bool ObjectLoader<T>::has_object(const std::string & object)
{
  return handles_.count(object) != 0;
}

template<typename T>
bool ObjectLoader<T>::has_symbol(const std::string & object, const std::string & symbol)
{
  return has_object(object) && lt_dlsym(handles_[object], symbol.c_str()) != nullptr;
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
void ObjectLoader<T>::load_libraries(const std::vector<std::string> & paths)
{
  Loader::load_libraries(paths, handles_);
  for(const auto & h : handles_)
  {
    if(deleters_.count(h.first) == 0)
    {
      void * sym = lt_dlsym(h.second, "destroy");
      if(sym == nullptr)
      {
        LOG_ERROR("Symbol destroy not found in " << lt_dlgetinfo(h.second)->filename << std::endl << lt_dlerror())
        throw(LoaderException("Required symbol not found"));
      }
      deleters_[h.first] = ObjectDeleter(sym);
    }
  }
}

template<typename T>
void ObjectLoader<T>::clear()
{
  handles_.clear();
}

template<typename T>
template<typename... Args>
std::shared_ptr<T> ObjectLoader<T>::create_object(const std::string & name, const Args & ... args)
{
  if(!has_object(name))
  {
    LOG_ERROR("Requested creation of object named " << name << " which has not been loaded")
    throw(LoaderException("No such object"));
  }
  void * sym = lt_dlsym(handles_[name], "create");
  if(sym == nullptr)
  {
    LOG_ERROR("Symbol create not found in " << lt_dlgetinfo(handles_[name])->filename << std::endl << lt_dlerror())
    throw(LoaderException("create symbol not found"));
  }
  const char * err = lt_dlerror();
  if(err != nullptr)
  {
    LOG_ERROR("Failed to resolve create symbol in " << lt_dlgetinfo(handles_[name])->filename << std::endl << err)
    throw(LoaderException("symbol resolution failed"));
  }
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wpedantic"
  std::function<T*(const Args & ...)> create_fn = (T*(*)(const Args & ...))(sym);
  #pragma GCC diagnostic pop
  T * ptr = sandbox_function_call(create_fn, args...);
  if(ptr == nullptr)
  {
    LOG_ERROR("Call to create for object " << name << " failed")
    throw(LoaderException("Create call failed"));
  }
  return std::shared_ptr<T>(ptr, deleters_[name]);
}

} // namespace mc_rtc
