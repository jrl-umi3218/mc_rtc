/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/utils_api.h>

#include <functional>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

namespace mc_rtc
{
namespace datastore
{

template<typename T>
bool is_valid_hash(std::size_t h)
{
  return h == typeid(T).hash_code();
}

template<typename T, typename U, typename... Args>
bool is_valid_hash(std::size_t h)
{
  return is_valid_hash<T>(h) || is_valid_hash<U, Args...>(h);
}

/** A generic data store */
struct MC_RTC_UTILS_DLLAPI DataStore
{
  DataStore() = default;
  DataStore(const DataStore &) = delete;
  DataStore & operator=(const DataStore &) = delete;
  DataStore(DataStore &&) = default;
  DataStore & operator=(DataStore &&) = default;

  template<typename T>
  const T & get(const std::string & name) const
  {
    auto it = datas_.find(name);
    if(it == datas_.end())
    {
      throw std::runtime_error("No such key");
    }
    auto & data = it->second;
    if(!data.same(typeid(T).hash_code()))
    {
      throw std::runtime_error("Not the same type as the stored type");
    }
    return *(reinterpret_cast<const T *>(data.buffer.get()));
  }

  template<typename T>
  T & get(const std::string & name)
  {
    auto it = datas_.find(name);
    if(it == datas_.end())
    {
      throw std::runtime_error("No such key");
    }
    auto & data = it->second;
    if(!data.same(typeid(T).hash_code()))
    {
      throw std::runtime_error("Not the same type as the stored type");
    }
    return *(reinterpret_cast<T *>(data.buffer.get()));
  }

  template<typename T, typename... ArgsT, typename... Args>
  void make(const std::string & name, Args &&... args)
  {
    auto & data = datas_[name];
    if(data.buffer)
    {
      data.destroy(data);
    }
    data.buffer.reset(new uint8_t[sizeof(T)]);
    new(data.buffer.get()) T(std::forward<Args>(args)...);
    data.same = &is_valid_hash<T, ArgsT...>;
    data.destroy = [](Data & self) { reinterpret_cast<T *>(self.buffer.get())->~T(); };
  }

private:
  struct Data
  {
    /** Hold the data */
    std::unique_ptr<uint8_t[]> buffer;
    /** Check requested type */
    bool (*same)(std::size_t);
    /** Call destructor and delete the buffer */
    void (*destroy)(Data &);
    /** Destructor */
    ~Data()
    {
      if(buffer)
      {
        destroy(*this);
      }
    }
  };
  std::unordered_map<std::string, Data> datas_;
};

} // namespace datastore
} // namespace mc_rtc
