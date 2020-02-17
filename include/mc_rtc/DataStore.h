/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/logging.h>
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

/**
 * @brief Generic data store
 *
 * This class allows to store and retrieve C++ objects. It is intended to be
 * used as a way to share arbitrary data between various parts of the framework
 * (states, plugins, observers, etc...)
 *
 * \code{cpp}
 * DataStore store;
 * // Creates a vector of 4 double with value 42 on the datastore
 * store.make<std::vector<double>>("Data", 4, 42);
 * ...
 * auto & data = store.get<std::vector<double>>("Data");
 * // Use the object
 * data[3] = 0;
 * ...
 * // Get another reference to the object
 * auto & data2 = store.get<std::vector<double>>("Data");
 * data2.push_back(0);
 * // vector now has size 5
 * LOG_INFO(data.size());
 * \endcode
 *
 * When retrieving an object using get<Type>, checks are performed to ensure that
 * the stored type and Type are compatible.
 *
 * To handle inheritance, you need to explicitely provide the class hierarchy:
 * \code{cpp}
 * struct A {};
 * struct B : public A {};
 * // Creating an inherited object and checking virtual inheritance
 * store.make<B, A>("data");
 * auto & base = store.get<A>("data");
 * auto & derived = store.get<B>("data");
 * \endcode
 */
struct MC_RTC_UTILS_DLLAPI DataStore
{
  DataStore() = default;
  DataStore(const DataStore &) = delete;
  DataStore & operator=(const DataStore &) = delete;
  DataStore(DataStore &&) = default;
  DataStore & operator=(DataStore &&) = default;

  /**
   * @brief Checks if an object is in the datastore
   *
   * @param name Name of the stored object
   *
   * @return true if the object is in the datastore
   */
  bool has(const std::string & name) const
  {
    return datas_.find(name) != datas_.end();
  }

  /**
   * @brief Get a reference to an object on the datastore
   * @param name Name of the stored oject
   * @return Reference to the stored object
   *
   * \note T must have the same type as defined when creating the object using
   * \ref make
   */
  template<typename T>
  T & get(const std::string & name)
  {
    auto it = datas_.find(name);
    if(it == datas_.end())
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "[" << name_ << "] No key \"" << name << "\"");
    }
    auto & data = it->second;
    if(!data.same(typeid(T).hash_code()))
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "[" << name_ << "] Object for key \"" << name
                                                  << "\" does not have the same type as the stored type");
    }
    return *(reinterpret_cast<T *>(data.buffer.get()));
  }

  /** @brief const variant of \ref get */
  template<typename T>
  const T & get(const std::string & name) const
  {
    return get<T>(name);
  }

  /**
   * @brief Assign value from the datastore if it exists, leave value unchanged
   * otherwise
   *
   * @param name Name of the stored data
   * @param data External object to modify if there is stored data in the
   * datastore
   */
  template<typename T>
  void assign(const std::string & name, T & data)
  {
    if(has(name))
    {
      data = get<T>(name);
    }
  }

  /**
   * @brief Creates an object on the datastore and returns a reference to it
   *
   * @param name Name of the stored object.
   * @param args Parameters to be passed to the object's constructor
   *
   * @return A reference to the constructed object
   *
   * \anchor make
   */
  template<typename T, typename... ArgsT, typename... Args>
  T & make(const std::string & name, Args &&... args)
  {
    auto & data = datas_[name];
    if(data.buffer)
    {
      data.destroy(data);
      LOG_ERROR_AND_THROW(std::runtime_error,
                          "[" << name_ << "] An object named " << name << " already exists on the datastore.");
    }
    data.buffer.reset(new uint8_t[sizeof(T)]);
    new(data.buffer.get()) T(std::forward<Args>(args)...);
    data.same = &is_valid_hash<T, ArgsT...>;
    data.destroy = [](Data & self) { reinterpret_cast<T *>(self.buffer.get())->~T(); };
    return *(reinterpret_cast<T *>(data.buffer.get()));
  }

  /**
   * @brief Creates an object on the datastore using initializer_list
   * initialization and returns a reference to it
   *
   * @param name Name of the stored object.
   * @param args Parameters to be passed to the object's using initializer_list
   * initialization.
   *
   * @return A reference to the constructed object
   */
  template<typename T, typename... ArgsT, typename... Args>
  T & make_initializer(const std::string & name, Args &&... args)
  {
    auto & data = datas_[name];
    if(data.buffer)
    {
      data.destroy(data);
      LOG_ERROR_AND_THROW(std::runtime_error,
                          "[" << name_ << "] An object named " << name << " already exists on the datastore.");
    }
    data.buffer.reset(new uint8_t[sizeof(T)]);
    new(data.buffer.get()) T{std::forward<Args>(args)...};
    data.same = &is_valid_hash<T, ArgsT...>;
    data.destroy = [](Data & self) { reinterpret_cast<T *>(self.buffer.get())->~T(); };
    return *(reinterpret_cast<T *>(data.buffer.get()));
  }

  /**
   * @brief Convenience function that creates an object on the datastore if it
   * does not already exist, or assign the passed values
   *
   * @param name Name of the stored object to create or modify
   * @param args Arguments passed for creation of the object
   *
   * @return The assigned object
   *
   * \anchor make_or_assign
   */
  template<typename T, typename... ArgsT, typename... Args>
  T & make_or_assign(const std::string & name, const Args &&... args)
  {
    if(has(name))
    {
      auto & data = get<T>(name);
      data = T(args...);
      return data;
    }
    else
    {
      return make<T>(name, args...);
    }
  }

  /**
   * @brief Convenience function that creates or assigns an object on the
   * datastore using list initialization.
   *
   * @param name Name of the stored object to create or modify
   * @param args Arguments passed for creation of the object. The object is
   * constructed using list initialization.
   *
   * @return The assigned object
   *
   * \see make_or_assign
   */
  template<typename T, typename... ArgsT, typename... Args>
  T & make_initializer_or_assign(const std::string & name, const Args &&... args)
  {
    if(has(name))
    {
      auto & data = get<T>(name);
      data = T{args...};
      return data;
    }
    else
    {
      return make_initializer<T>(name, args...);
    }
  }

  /**
   * @brief Removes an object from the datastore
   * @param name Name of the stored object
   */
  void remove(const std::string & name)
  {
    auto it = datas_.find(name);
    if(it == datas_.end())
    {
      LOG_ERROR("[" << name_ << "] Failed to remove element \"" << name << "\" (element does not exist)");
      return;
    }
    if(it->second.buffer)
    {
      it->second.destroy(it->second);
    }
    datas_.erase(it);
  }

  /**
   * @brief Name of this datastore
   */
  const std::string & name() const
  {
    return name_;
  }

  /**
   * @brief Sets this datastore's name
   *
   * @param name Name for the datastore
   */
  void name(const std::string & name)
  {
    name_ = name;
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
  std::string name_ = "DataStore";
};

} // namespace datastore
} // namespace mc_rtc
