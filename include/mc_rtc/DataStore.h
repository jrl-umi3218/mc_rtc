/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/logging.h>
#include <mc_rtc/utils_api.h>

#include <functional>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace mc_rtc
{

namespace internal
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

MC_RTC_UTILS_DLLAPI std::string demangle(const char * name);

template<typename T>
std::string type_name()
{
  return demangle(typeid(T).name());
}

/** Extract return type and argument types from a lambda by accessing ::operator() */
template<typename T>
struct lambda_traits : public lambda_traits<decltype(&T::operator())>
{
};

/** Specialization that matches non-mutable lambda */
template<typename C, typename RetT, typename... Args>
struct lambda_traits<RetT (C::*)(Args...) const>
{
  using fn_t = std::function<RetT(Args...)>;
};

/** Specialization that matches mutable lambda */
template<typename C, typename RetT, typename... Args>
struct lambda_traits<RetT (C::*)(Args...)>
{
  using fn_t = std::function<RetT(Args...)>;
};

/** Bend usual C++ rules to deduct the argument types in call
 *
 * Normally when passing an lvalue to a template function, T && will be T &. In
 * our case, for arithmetic types (typically double) we far more often want T
 * so this traits changes the deduction rule for those types
 */
template<typename T>
struct args_t
{
  using decay_t = typename std::decay<T>::type;
  static constexpr bool is_arithmetic = std::is_arithmetic<decay_t>::value;
  using type = typename std::conditional<is_arithmetic, decay_t, T>::type;
};

/** Allocator<T> is std::allocator<T> for usual types */
template<typename T, typename = void>
struct Allocator : public std::allocator<T>
{
};

/** Allocator<T> is Eigen::aligned_allocator for EIGEN_MAKE_ALIGNED_OPERATOR_NEW types */
template<typename T>
struct Allocator<T, typename T::eigen_aligned_operator_new_marker_type> : public Eigen::aligned_allocator<T>
{
};

} // namespace internal

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
struct DataStore
{
  DataStore() = default;
  DataStore(const DataStore &) = delete;
  DataStore & operator=(const DataStore &) = delete;
  DataStore(DataStore &&) = default;
  DataStore & operator=(DataStore &&) = default;

  /**
   * @brief Checks whether an object is in the datastore
   *
   * @param name Name of the stored object
   *
   * @return true when the object is in the datastore
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
   * @throws std::runtime_error when the type of T does not match the one
   * defined upon creation.
   *
   * @ref make
   */
  template<typename T>
  T & get(const std::string & name)
  {
    return const_cast<T &>(get_<T>(name));
  }

  /** @brief const variant of \ref get */
  template<typename T>
  const T & get(const std::string & name) const
  {
    return get_<T>(name);
  }

  /**
   * @brief Assign value from the datastore if it exists, leave value unchanged
   * otherwise
   *
   * @param name Name of the stored data
   * @param data Reference to the external object to assign with the stored data
   */
  template<typename T>
  void get(const std::string & name, T & data)
  {
    auto it = datas_.find(name);
    if(it != datas_.end())
    {
      data = safe_cast<T>(it->second, name);
    }
  }

  /**
   * @brief Gets a value from the datastore if it exists or a default value
   * otherwise
   *
   * @param name Name of the stored data
   * @param defaultValue Default value used if the data is not in the datastore
   *
   * @return Stored data if it exists, defaultValue otherwise.
   */
  template<typename T>
  const T & get(const std::string & name, const T & defaultValue) const
  {
    auto it = datas_.find(name);
    if(it != datas_.end())
    {
      return safe_cast<T>(it->second, name);
    }
    return defaultValue;
  }

  /**
   * @brief Copies an object to an existing datastore object
   *
   * @param name Name of the stored object
   * @param data Data to copy on the datastore
   *
   * \see get(std::string)
   */
  template<typename T>
  void assign(const std::string & name, const T & data)
  {
    get<T>(name) = data;
  }

  /**
   * @brief Creates an object on the datastore and returns a reference to it
   *
   * @param name Name of the stored object.
   * @param args Parameters to be passed to the object's constructor
   *
   * @return A reference to the constructed object
   *
   * @throws std::runtime_error if an object with the same name already exists
   * in the datastore.
   *
   * \anchor make
   */
  template<typename T, typename... ArgsT, typename... Args>
  T & make(const std::string & name, Args &&... args)
  {
    auto & data = datas_[name];
    if(data.buffer)
    {
      LOG_ERROR_AND_THROW(std::runtime_error,
                          "[" << name_ << "] An object named " << name << " already exists on the datastore.");
    }
    data.allocate<T>(name_, name);
    new(data.buffer.get()) T(std::forward<Args>(args)...);
    return data.setup<T, ArgsT...>();
  }

  /**
   * @brief Creates an object on the datastore and returns a reference to it
   *
   * This overload should only work with lambda-like objects and transform the lambda into an equivalent std::function
   *
   * @param name Name of the stored object
   * @param fn Function that will be stored in the datastore
   *
   * @return A reference to the constructed object
   */
  template<typename T>
  auto make_call(const std::string & name, T fn) -> typename internal::lambda_traits<T>::fn_t &
  {
    using fn_t = typename internal::lambda_traits<T>::fn_t;
    return make<fn_t>(name, fn_t(fn));
  }

  /**
   * @brief Creates an object on the datastore using list initialization and returns a reference to it
   *
   * @param name Name of the stored object.
   * @param args Parameters to be passed to the object using list initialization
   *
   * @return A reference to the constructed object
   */
  template<typename T, typename... ArgsT, typename... Args>
  T & make_initializer(const std::string & name, Args &&... args)
  {
    auto & data = datas_[name];
    if(data.buffer)
    {
      LOG_ERROR_AND_THROW(std::runtime_error,
                          "[" << name_ << "] An object named " << name << " already exists on the datastore.");
    }
    data.allocate<T>(name_, name);
    new(data.buffer.get()) T{std::forward<Args>(args)...};
    return data.setup<T, ArgsT...>();
  }

  /** @brief Calls a function that was registered in the datastore and returns
   * this call result
   *
   * This is syntactic sugar for getting the function object and calling it
   *
   * In this version you must specify the argument types for the function you
   * are calling. This is especially useful when the types deduced from the
   * arguments do not match the function type (typically passing a non-const
   * reference to a function expecting a const reference.
   *
   * @param name Name of the stored function
   *
   * @tparam FunArgsT Types of arguments expected by the function
   *
   * @param args Arguments passed to the function
   *
   */
  template<typename RetT,
           typename... FuncArgsT,
           typename... ArgsT,
           typename std::enable_if<sizeof...(FuncArgsT) == sizeof...(ArgsT)
                                       && !std::is_same<std::tuple<FuncArgsT...>, std::tuple<ArgsT...>>::value,
                                   int>::type = 0>
  RetT call(const std::string & name, ArgsT &&... args) const
  {
    return safe_call<RetT, FuncArgsT...>(name, std::forward<ArgsT>(args)...);
  }

  /** @brief Calls a function that was registered in the datastore and returns
   * this call result
   *
   * This is syntaxic sugar for getting the function object and calling it
   *
   * This version deduces the functions' arguments from the arguments passed to
   * the call. This follows C++ rules for deducing types from universal
   * reference collapsing *except* for arithmetic types where the type is
   * deduced to be of value type.
   *
   * If you need to call a callback that takes an arithmetic type by reference
   * then you must specify the arguments' type explicitely
   *
   * @param name Name of the stored object
   *
   * @params args Arguments passed to the function
   *
   */
  template<typename RetT = void, typename... ArgsT>
  RetT call(const std::string & name, ArgsT &&... args) const
  {
    return safe_call<RetT, typename internal::args_t<ArgsT>::type...>(name, std::forward<ArgsT>(args)...);
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
    Data() = default;
    Data(const Data &) = delete;
    Data & operator=(const Data &) = delete;
    Data(Data &&) = default;
    Data & operator=(Data &&) = default;

    /** Hold the data */
    std::unique_ptr<uint8_t[]> buffer;
    /** Return the stored type name */
    std::string (*type)();
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

    template<typename T>
    void allocate(const std::string & name_, const std::string & name)
    {
      if(buffer)
      {
        LOG_ERROR_AND_THROW(std::runtime_error,
                            "[" << name_ << "] An object named " << name << " already exists on the datastore.");
      }
      buffer.reset(reinterpret_cast<uint8_t *>(internal::Allocator<T>().allocate(1)));
    }

    template<typename T, typename... ArgsT>
    T & setup()
    {
      this->type = &internal::type_name<T>;
      this->same = &internal::is_valid_hash<T, ArgsT...>;
      this->destroy = [](Data & self) {
        T * p = reinterpret_cast<T *>(self.buffer.release());
        p->~T();
        internal::Allocator<T>().deallocate(p, 1);
      };
      return *(reinterpret_cast<T *>(buffer.get()));
    }
  };

  template<typename T>
  const T & safe_cast(const Data & data, const std::string & name) const
  {
    if(!data.same(typeid(T).hash_code()))
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "[" << name_ << "] Object for key \"" << name
                                                  << "\" does not have the same type as the stored type. "
                                                  << "Stored " << data.type() << " but requested "
                                                  << internal::type_name<T>());
    }
    return *(reinterpret_cast<T *>(data.buffer.get()));
  }

  template<typename RetT, typename... FuncArgsT, typename... ArgsT>
  RetT safe_call(const std::string & name, ArgsT &&... args) const
  {
    const auto & data = get_data(name);
    using fn_t = std::function<RetT(FuncArgsT...)>;
    if(!data.same(typeid(fn_t).hash_code()))
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "[" << name_ << "] Function for key \"" << name
                                                  << "\" does not have the same signature as the requested one. "
                                                  << "Stored " << data.type() << " but requested "
                                                  << internal::type_name<fn_t>());
    }
    auto & fn = *(reinterpret_cast<fn_t *>(data.buffer.get()));
    return fn(std::forward<ArgsT>(args)...);
  }

  template<typename T>
  const T & get_(const std::string & name) const
  {
    return safe_cast<T>(get_data(name), name);
  }

  inline const Data & get_data(const std::string & name) const
  {
    const auto it = datas_.find(name);
    if(it == datas_.end())
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "[" << name_ << "] No key \"" << name << "\"");
    }
    return it->second;
  }

private:
  std::unordered_map<std::string, Data> datas_;
  std::string name_ = "DataStore";
};

} // namespace mc_rtc
