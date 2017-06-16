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
ObjectLoader<T>::ObjectLoader(const std::string & class_name, const std::vector<std::string> & paths, bool enable_sandbox, bool verbose)
: class_name(class_name),
  enable_sandbox(enable_sandbox),
  verbose(verbose)
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
  Loader::load_libraries(class_name, paths, handles_, verbose);
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
std::shared_ptr<T> ObjectLoader<T>::create_object(const std::string & name, const Args & ... args)
{
  if(!has_object(name))
  {
    LOG_ERROR("Requested creation of object named " << name << " which has not been loaded")
    throw(LoaderException("No such object"));
  }
  unsigned int args_passed = 1 + sizeof...(Args);
  unsigned int args_required = args_passed;
  void * args_required_sym = lt_dlsym(handles_[name], "create_args_required");
  if(args_required_sym)
  {
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic"
    auto create_args_required = (unsigned int(*)())(args_required_sym);
    #pragma GCC diagnostic pop
    args_required = create_args_required();
  }
  else
  {
    /* Discard error message */
    lt_dlerror();
  }
  if(args_passed != args_required)
  {
    LOG_ERROR(args_passed << " arguments passed to create function of " << name << " which expects " << args_required)
    throw(LoaderException("Missmatch arguments number"));
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
  std::function<T*(const std::string &, const Args & ...)> create_fn = (T*(*)(const std::string &, const Args & ...))(sym);
  #pragma GCC diagnostic pop
  T * ptr = nullptr;
  if(enable_sandbox)
  {
    ptr = sandbox_function_call(create_fn, name, args...);
  }
  else
  {
    ptr = no_sandbox_function_call(create_fn, name, args...);
  }
  if(ptr == nullptr)
  {
    LOG_ERROR("Call to create for object " << name << " failed")
    throw(LoaderException("Create call failed"));
  }
  return std::shared_ptr<T>(ptr, deleters_[name]);
}

} // namespace mc_rtc
