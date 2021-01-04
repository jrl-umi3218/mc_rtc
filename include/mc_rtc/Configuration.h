/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/MessagePackBuilder.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <mc_rbdyn/rpy_utils.h>
#include <Eigen/Core>
#include <array>
#include <exception>
#include <map>
#include <mc/rtc/deprecated.hh>
#include <memory>
#include <set>
#include <spdlog/fmt/fmt.h>
#include <string>
#include <unordered_set>
#include <vector>

namespace mc_rtc
{

struct MC_RTC_UTILS_DLLAPI ConfigurationArrayIterator;
struct MC_RTC_UTILS_DLLAPI Configuration;

/** This structure should be specialized to implement serialization for a new type.
 *
 * load return type should be T
 *
 * save return type should be mc_rtc::Configuration
 */
template<typename T>
struct ConfigurationLoader
{
  static void load(const mc_rtc::Configuration &) {}

  static void save(const T &) {}
};

template<>
struct ConfigurationLoader<void>
{
  static void load(const mc_rtc::Configuration &) {}
  static void save() {}
};

namespace internal
{
/** Helper trait to determine whether:
 * T ConfigurationLoader<T>::load(const mc_rtc::Configuration &);
 * is a valid function or not
 */
struct _has_configuration_load_object
{
  template<typename T,
           typename std::enable_if<
               std::is_same<decltype(ConfigurationLoader<T>::load(std::declval<const Configuration &>())), T>::value,
               int>::type = 0>
  static std::true_type test(T * p);

  template<typename T>
  static std::false_type test(...);
};

template<typename T>
struct has_configuration_load_object : decltype(_has_configuration_load_object::test<T>(nullptr))
{
};

/** Helper trait to determine whether:
 * mc_rtc::Configuration mc_rtc::ConfigurationLoader<T>::save(const T&, Args ...);
 * is a valid construct or not
 */
struct _has_configuration_save_object
{
  template<typename T,
           typename... Args,
           typename std::enable_if<
               std::is_same<decltype(ConfigurationLoader<T>::save(std::declval<const T &>(), std::declval<Args>()...)),
                            Configuration>::value,
               int>::type = 0>
  static std::true_type test(T * p);

  template<typename T, typename... Args>
  static std::false_type test(...);
};

template<typename T, typename... Args>
struct has_configuration_save_object : decltype(_has_configuration_save_object::test<T, Args...>(nullptr))
{
};
} // namespace internal

/*! \brief Simplify access to values hold within a JSON file
 *
 * Configuration values can be accessed using a key system, type
 * conversion are ensured by the class.
 */
struct MC_RTC_UTILS_DLLAPI Configuration
{
private:
  /*! \brief Implementation details
   *
   * This structure is meant to hide the JSON library used by mc_rtc
   * while allowing the template method implementation.
   */
  struct MC_RTC_UTILS_DLLAPI Json
  {
    /** True if the underlying value is an array */
    bool isArray() const;
    /** Size of the array, 0 if the array is empty or the value is not an array */
    size_t size() const;
    /** Access element at the provided index
     *
     * \throws If idx >= size()
     */
    Json operator[](size_t idx) const;
    /** True if the underlying value is an object */
    bool isObject() const;
    /** Key of the object, empty if the object is or if the value is not an object */
    std::vector<std::string> keys() const;
    /** Access element at the provided key
     *
     * \throws If key does not belong in keys()
     */
    Json operator[](const std::string & key) const;
    /** True if the value is numeric */
    bool isNumeric() const;
    /** Access the value as a double
     *
     * \throws If isNumeric() is false
     */
    double asDouble() const;
    /** Output the path from the root of the document to this value
     *
     * \note This requires searching through the whole document for this value and is not very efficient
     */
    void path(std::string & out) const;
    /** Actually a RapidJSON value */
    void * value_;
    /** Actually a RapidJSON document (root) */
    std::shared_ptr<void> doc_;
  };

public:
  /*! \brief Exception thrown by this class when something bad occurs
   *
   * The exception message is generated when accessed via what() or msg(). It is printed when the exception is deleted
   * and builds a path from the root of the document to the source of the error in order to help the user to find the
   * source of the error. This operation can be expensive. If you do not need the message (e.g. you are attempting
   * various conversions) then you should call silence() to prevent the generation of the message.
   */
  struct MC_RTC_UTILS_DLLAPI Exception : public std::exception
  {
    /*! \brief Constructor
     *
     * \param msg Exception message
     *
     * \param v Json value that was the source of the exception
     */
    Exception(const std::string & msg, const Json & v);

    /*! \brief Constructor
     *
     * \param msg Exception message
     *
     * \param c Configuration object that was the source of the exception
     */
    Exception(const std::string & msg, const Configuration & c) : Exception(msg, c.v) {}

    ~Exception() noexcept;

    /** Returns the error message */
    virtual const char * what() const noexcept override;

    /** Empty the error message */
    void silence() const noexcept;

    /** Returns the error message */
    const std::string & msg() const noexcept;

  private:
    mutable std::string msg_;
    mutable Json v_;
  };

  /*! \brief Deprecated, see has
   *
   * \param key The key to test
   *
   * \returns True if key is part of the configuration
   */
  MC_RTC_DEPRECATED bool isMember(const std::string & key) const;

  /*! \brief Check if the key is part of the conf
   *
   * \param key The key to test
   *
   * \returns True if key is part of the configuration
   */
  bool has(const std::string & key) const;

  /*! \brief Cast to bool
   *
   * \throws If the underlying value does not hold a boolean
   */
  operator bool() const;

  /*! \brief Cast to int
   *
   * Strictly for int-typed entries
   *
   * \throws If the underlying value does not hold an int
   */
  operator int() const;

  /*! \brief Cast to unsigned int
   *
   * Int entries that are strictly positive will be treated as
   * unsigned int entries
   *
   * \throws If the underlying value does not hold an unsigned int
   */
  operator unsigned int() const;

  /*! \brief Cast to int64_t
   *
   * Strictly for int64_t-typed entries
   *
   * \throws If the underlying value does not hold an int64_t
   */
  operator int64_t() const;

  /*! \brief Cast to uint64_t
   *
   * Int entries that are strictly positive will be treated as
   * uint64_t entries
   *
   * \throws If the underlying value does not hold an uint64_t
   */
  operator uint64_t() const;

  /*! \brief Cast to double
   *
   * Will actually convert any numeric entry into a double
   *
   * \throws If the underlying value does not hold a numeric value
   */
  operator double() const;

  /*! \brief Cast to a string
   *
   * \note You may need explicit casting in some cases, e.g.
   * writing a = config("entry") where a is a std::string instance
   * requires explicit casting to compile. Alternatively, in this
   * scenario, one can use config("entry", a) without explicit
   * casting.
   *
   * \throws If the underlying value does not hold a string
   */
  operator std::string() const;

  /*! \brief Retrieve as a Eigen::Vector2d instance
   *
   * \throws If the underlying value does not hold a numeric
   * sequence of size 2
   */
  operator Eigen::Vector2d() const;

  /*! \brief Retrieve as a Eigen::Vector3d instance
   *
   * \throws If the underlying value does not hold a numeric
   * sequence of size 3
   */
  operator Eigen::Vector3d() const;

  /*! \brief Retrieve as a Eigen::Vector6d instance
   *
   * \throws If the underlying value does not hold a numeric
   * sequence of size 6
   */
  operator Eigen::Vector6d() const;

  /*! \brief Retrieve as a Eigen::VectorXd instance
   *
   * If the underlying value holds an empty sequence, this
   * functions returns an empty (size 0) vector
   *
   * \throws If the underlying value does not hold a numeric
   * sequence
   */
  operator Eigen::VectorXd() const;

  /*! \brief Retrieve as a Eigen::Quaterniond instance
   *
   * \note The sequence is normalized
   *
   * \throws If the underlying value does not hold a numeric
   * sequence of size 4
   */
  operator Eigen::Quaterniond() const;

  /*! \brief Retrieve as a Eigen::Matrix3d instance
   * \anchor Matrix3d_operator
   *
   * \throws If the underlying value does not hold a numeric sequence of size 9
   */
  operator Eigen::Matrix3d() const;

  /*! \brief Retrieve as a Eigen::Matrix6d instance
   *
   * \throws If the underlying value does not hold a numeric sequence of size 36
   */
  operator Eigen::Matrix6d() const;

  /*! \brief Retrieve as a Eigen::MatrixXd instance
   *
   * \throws If the underlying value does not hold an array of array of numeric values
   */
  operator Eigen::MatrixXd() const;

  /*! \brief Retrieve as an sva::PTransformd
   *
   * \throws If the underlying value is not an object with rotation and/or translation members
   */
  operator sva::PTransformd() const;

  /*! \brief Retrieve as an sva::ForceVecd
   *
   * \throws If the underlying value is not an object with couple and force members
   */
  operator sva::ForceVecd() const;

  /*! \brief Retrieve as an sva::MotionVecd
   *
   * \throws If the underlying value is not an object with angular and linear members
   */
  operator sva::MotionVecd() const;

  /*! \brief Retrieve as an sva::ImpedanceVecd
   *
   * \throws If the underlying value is not an object with angular and linear members
   */
  operator sva::ImpedanceVecd() const;

  /*! \brief Retrieve a vector instance
   *
   * \throws If the underlying value does not hold an array or if
   * any member of the array does not meeting the requirement of
   * the vector elements' type
   */
  template<class T, class A>
  operator std::vector<T, A>() const
  {
    if(v.isArray())
    {
      std::vector<T, A> ret;
      for(size_t i = 0; i < v.size(); ++i)
      {
        ret.push_back(Configuration(v[i]));
      }
      return ret;
    }
    else
    {
      throw Configuration::Exception("Stored Json value is not a vector", v);
    }
  }

  /*! \brief Retrieve an array instance
   *
   * \throws If the underlying value does not hold an array of the correct
   * size or if any member of the array does not meet the requirements of the
   * array elements' type.
   */
  template<class T, std::size_t N>
  operator std::array<T, N>() const
  {
    if(v.isArray() && v.size() == N)
    {
      std::array<T, N> ret;
      for(size_t i = 0; i < N; ++i)
      {
        ret[i] = Configuration(v[i]);
      }
      return ret;
    }
    else
    {
      throw Configuration::Exception("Stored Json value is not an array or its size is incorrect", v);
    }
  }

  /*! \brief Retrieve a pair instance
   *
   * \throws If the underlying value does not hold an array of size 2 or if
   * the member of the array do not meet the requirement of the pair type
   */
  template<class T1, class T2>
  operator std::pair<T1, T2>() const
  {
    if(v.isArray() && v.size() == 2)
    {
      return std::make_pair<T1, T2>(Configuration(v[0]), Configuration(v[1]));
    }
    else
    {
      throw Configuration::Exception("Stored Json value is not an array of size 2", v);
    }
  }

  /*! \brief Retrieve a string-indexed map instance
   *
   * \throws If the underlying value is not an object or if the member
   * of the object do not meet the requiremenent of the value type.
   */
  template<typename T, class C, class A>
  operator std::map<std::string, T, C, A>() const
  {
    if(v.isObject())
    {
      std::map<std::string, T, C, A> ret;
      auto keys = v.keys();
      assert(std::set<std::string>(keys.begin(), keys.end()).size() == keys.size());
      for(const auto & k : keys)
      {
        T value = Configuration(v[k]);
        ret[k] = value;
      }
      return ret;
    }
    else
    {
      throw Configuration::Exception("Stored Json value is not an object", v);
    }
  }

  /*! \brief Retrieve a set of objects
   *
   * \throws If the underlying value is not an array, if the member of the
   * array do not meet the requirement of the set key type or if the elements
   * in the array are not unique
   */
  template<typename T, typename C = std::less<T>, typename A = std::allocator<T>>
  operator std::set<T, C, A>() const
  {
    if(v.isArray())
    {
      std::set<T, C, A> ret;
      for(size_t i = 0; i < v.size(); ++i)
      {
        auto ins = ret.insert(Configuration(v[i]));
        if(!ins.second)
        {
          throw Configuration::Exception("Stored Json set does not hold unique values", v);
        }
      }
      return ret;
    }
    else
    {
      throw Configuration::Exception("Stored Json value is not an array", v);
    }
  }

  /*! \brief Retrieve an unordered set of objects
   *
   * \throws If the underlying value is not an array, if the member of the
   * array do not meet the requirement of the set key type or if the elements
   * in the array are not unique
   */
  template<typename T, typename H = std::hash<T>, typename E = std::equal_to<T>, typename A = std::allocator<T>>
  operator std::unordered_set<T, H, E, A>() const
  {
    if(v.isArray())
    {
      std::unordered_set<T, H, E, A> ret;
      for(size_t i = 0; i < v.size(); ++i)
      {
        auto ins = ret.insert(Configuration(v[i]));
        if(!ins.second)
        {
          throw Configuration::Exception("Stored Json set does not hold unique values", v);
        }
      }
      return ret;
    }
    else
    {
      throw Configuration::Exception("Stored Json value is not an array", v);
    }
  }

  /*! \brief User-defined conversions
   *
   * Requires:
   * - T mc_rtc::ConfigurationLoader<T>::load(const mc_rtc::Configuration &) should exist
   */
  template<typename T, typename std::enable_if<internal::has_configuration_load_object<T>::value, int>::type = 0>
  operator T() const
  {
    return ConfigurationLoader<T>::load(*this);
  }

  /*! \brief Creates an empty configuration */
  Configuration();

  /*! \brief Constructor using a file path
   *
   * \param path Path to the configuration file
   *
   */
  Configuration(const std::string & path);

  /*! \brief Constructor using a file path (C-style string)
   *
   * \param path Path to the configuration file
   *
   */
  Configuration(const char * path);

  /*! \brief Returns a Configuration with an array as root entry
   *
   * This is not valid standard JSON
   */
  static Configuration rootArray();

  /*! \brief Static constructor to load from JSON data
   *
   * \param data JSON data to load
   *
   */
  static Configuration fromData(const std::string & data);

  /*! \brief Static constructor to load from JSON data (C overload)
   *
   * \param data JSON data to load
   *
   */
  static Configuration fromData(const char * data);

  /*! \brief Static constructor to load from YAML data
   *
   * \param data YAML data to load
   *
   */
  static Configuration fromYAMLData(const std::string & data);

  /*! \brief Static constructor to load from YAML data (C overload)
   *
   * \param data YAML data to load
   *
   */
  static Configuration fromYAMLData(const char * data);

  /*! \brief Static constructor to load from MessagePack data
   *
   * \param data MessagePack data to load
   *
   * \param size Size of data
   *
   */
  static Configuration fromMessagePack(const char * data, size_t size);

  /*! \brief Load more data into the configuration
   *
   * For any key existing in both objects:
   * - (*this)(key) is overwritten for values and arrays
   * - config(key) is loaded into (*this)(key) for objects
   *
   * \param path Path to the configuration file
   *
   */
  void load(const std::string & path);

  /*! \brief Load data from another Configuration object
   *
   * For any key existing in both objects:
   * - (*this)(key) is overwritten for values and arrays
   * - if config(key) and (*this)(key) are objects, config(key) is
   *   loaded into (*this)(key)
   * - otherwise, config(key) overwrites (*this)(key)
   *
   * \param config The configuration object
   *
   */
  void load(const mc_rtc::Configuration & config);

  /*! \brief Load data from a JSON string
   *
   * Same rules apply as ::load methods
   *
   * \param data JSON data to load
   *
   */
  void loadData(const std::string & data);

  /*! \brief Load data from a YAML string
   *
   * Same rules apply as ::load methods
   *
   * \param data YAML data to load
   *
   */
  void loadYAMLData(const std::string & data);

  /*! \brief Save the configuration to a file.
   *
   * If the path extension is yaml or yml then save in YAML format
   *
   * \param path Path to the configuration file
   *
   * \param pretty Writes a human-readable file, defaults to true
   *
   */
  void save(const std::string & path, bool pretty = true) const;

  /*! \brief Dump the configuration into a string.
   *
   * \param pretty Writes a human-readable string, defaults to false
   *
   * \param yaml Writes YAML instead of JSON, defaults to false
   *
   */
  std::string dump(bool pretty = false, bool yaml = false) const;

  /*! \brief Convert to MessagePack
   *
   * \param data Will hold the message data
   *
   * \returns The size of the message, this is different from data.size()
   *
   */
  size_t toMessagePack(std::vector<char> & data) const;

  /*! \brief Append to an existing MessagePackBuilder
   *
   * The whole configuration will be stored in the MessagePack being built
   *
   * \param builder MessagePackBuilder instance
   *
   */
  void toMessagePack(MessagePackBuilder & builder) const;

  /*! \brief Returns a Entry value stored within the
   * configuration
   *
   * \param key The key used to store the value
   *
   * \returns The Entry value stored in the
   * configuration in the key entry
   *
   * \throws If key is not stored in the Configuration
   */
  Configuration operator()(const std::string & key) const;

  /*! \brief Returns true if the underlying array or underlying object is empty */
  bool empty() const;

  /*! \brief If the stored value is an array, returns its size, otherwise
   * returns 0
   *
   * \returns Size of the underlying array
   *
   */
  size_t size() const;

  /*! \brief If the stored value is an array, return a Configuration element
   * for the i-th element.
   *
   * \param i Access i-th element
   *
   * \throws If i >= size()
   */
  Configuration operator[](size_t i) const;

  /*! \brief Retrieve a given value from a JSON array
   *
   * Returns the default value if the index is too high or if the underlying value does not match the requested type.
   *
   * \param i Index to retrieve
   *
   * \param v The default value
   */
  template<typename T>
  T at(size_t i, const T & v) const
  {
    try
    {
      return (*this)[i];
    }
    catch(Exception & exc)
    {
      exc.silence();
      return v;
    }
  }

  /*! \brief Retrieve and store a given value stored within the
   * configuration
   * \anchor retrieve_and_store_template
   *
   * If they key is not stored in the Configuration, the value is
   * unchanged.
   *
   * \param key The key used to store the value
   *
   * \param v The value to retrieve
   *
   */
  template<typename T>
  void operator()(const std::string & key, T & v) const
  {
    try
    {
      v = (*this)(key);
    }
    catch(Exception & exc)
    {
      exc.silence();
    }
  }

  /*! \brief Retrieve a given value stored within the configuration with a
   * default value
   *
   * If the key is not stored in the Configuration or if the underyling value
   * does not match the requested type, the default value is returned.
   *
   * \param key The key used to store the value
   *
   * \param v The default value
   */
  template<typename T>
  T operator()(const std::string & key, const T & v) const
  {
    try
    {
      return (*this)(key);
    }
    catch(Exception & exc)
    {
      exc.silence();
      return v;
    }
  }

  /*! \brief Non-template version for C-style strings comparison
   *
   * \returns True if the comparison matches, false otherwise.
   */
  bool operator==(const char * rhs) const;

  /*! \brief Compare stored values with given value
   *
   * \returns True if the comparison matches, false otherwise.
   */
  template<typename T>
  bool operator==(const T & rhs) const
  {
    T lhs = *this;
    return lhs == rhs;
  }

  /*! \brief Add a null element to the configuration
   *
   * Overrides the existing value if it holds one for the given key
   *
   * \param key Key of the element
   *
   */
  void add_null(const std::string & key);

  /*! \brief Add a bool element to the Configuration
   *
   * Overrides the existing value if it holds one for the given key.
   *
   * \param key Key of the element
   *
   * \param value Value of the element
   */
  void add(const std::string & key, bool value);

  /*! \brief Add a int element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, int value);

  /*! \brief Add a unsigned int element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, unsigned int value);

  /*! \brief Add a int64_t element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, int64_t value);

  /*! \brief Add a uint64_t element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, uint64_t value);

  /*! \brief Add a double element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, double value);

  /*! \brief Add a std::string element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const std::string & value);

  /*! \brief Add a const char* element to the Configuration,
   * Behaves like std::string
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const char * value);

  /*! \brief Add a Eigen::Vector2d element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const Eigen::Vector2d & value);

  /*! \brief Add a Eigen::Vector3d element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const Eigen::Vector3d & value);

  /*! \brief Add a Eigen::Vector6d element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const Eigen::Vector6d & value);

  /*! \brief Add a Eigen::VectorXd element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const Eigen::VectorXd & value);

  /*! \brief Add a Eigen::Quaterniond element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const Eigen::Quaterniond & value);

  /*! \brief Add a Eigen::Matrix3d element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const Eigen::Matrix3d & value);

  /*! \brief Add a Eigen::Matrix6d element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const Eigen::Matrix6d & value);

  /*! \brief Add an Eigen::MatrixXd element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const Eigen::MatrixXd & value);

  /*! \brief Add an sva::PTransformd element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const sva::PTransformd & value);

  /*! \brief Add an sva::ForceVecd element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const sva::ForceVecd & value);

  /*! \brief Add an sva::MotionVecd element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const sva::MotionVecd & value);

  /*! \brief Add an sva::ImpedanceVecd element to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const sva::ImpedanceVecd & value);

  /*! \brief Add another Configuration to the Configuration
   *
   * \see add(const std::string&, bool)
   */
  void add(const std::string & key, const Configuration & value);

  /*! \brief Create an empty object in the Configuration
   *
   * Overwrite existing content if any.
   *
   * \param key Key to add
   */
  Configuration add(const std::string & key);

  /*! \brief Create an empty array in the Configuration
   *
   * Overwrite existing content if any.
   *
   * \param key Key to add
   *
   * \param size Size that is reserved for the array
   */
  Configuration array(const std::string & key, size_t size = 0);

  /*! \brief Insert a null value into an array
   *
   * \throws If the underlying Json value is not an array.
   */
  void push_null();

  /*! \brief Insert a bool element into an array
   *
   * \param value Value to add
   *
   * \throws If the underlying Json value is not an array.
   */
  void push(bool value);

  /*! \brief Insert a int element into an array
   *
   * \see push(bool);
   */
  void push(int value);

  /*! \brief Insert a unsigned int element into an array
   *
   * \see push(bool);
   */
  void push(unsigned int value);

  /*! \brief Insert a int64_t element int64_to an array
   *
   * \see push(bool);
   */
  void push(int64_t value);

  /*! \brief Insert a uint64_t element into an array
   *
   * \see push(bool);
   */
  void push(uint64_t value);

  /*! \brief Insert a double element into an array
   *
   * \see push(bool);
   */
  void push(double value);

  /*! \brief Insert a std::string element into an array
   *
   * \see push(bool);
   */
  void push(const std::string & value);

  /*! \brief Insert a const char * element into an array
   * Behaves like push(std::string)
   * \see push(bool);
   */
  void push(const char * value);

  /*! \brief Insert a Eigen::Vector2d element into an array
   *
   * \see push(bool);
   */
  void push(const Eigen::Vector2d & value);

  /*! \brief Insert a Eigen::Vector3d element into an array
   *
   * \see push(bool);
   */
  void push(const Eigen::Vector3d & value);

  /*! \brief Insert a Eigen::Vector6d element into an array
   *
   * \see push(bool);
   */
  void push(const Eigen::Vector6d & value);

  /*! \brief Insert a Eigen::VectorXd element into an array
   *
   * \see push(bool);
   */
  void push(const Eigen::VectorXd & value);

  /*! \brief Insert a Eigen::Quaterniond element into an array
   *
   * \see push(bool);
   */
  void push(const Eigen::Quaterniond & value);

  /*! \brief Insert a Eigen::Matrix3d element into an array
   *
   * \see push(bool);
   */
  void push(const Eigen::Matrix3d & value);

  /*! \brief Insert a Eigen::Matrix6d element into an array
   *
   * \see push(bool);
   */
  void push(const Eigen::Matrix6d & value);

  /*! \brief Insert a Eigen::MatrixXd element into an array
   *
   * \see push(bool);
   */
  void push(const Eigen::MatrixXd & value);

  /*! \brief Insert an sva::PTransformd element into an array
   *
   * \see push(bool)
   */
  void push(const sva::PTransformd & value);

  /*! \brief Insert an sva::ForceVecd element into an array
   *
   * \see push(bool)
   */
  void push(const sva::ForceVecd & value);

  /*! \brief Insert an sva::MotionVecd element into an array
   *
   * \see push(bool)
   */
  void push(const sva::MotionVecd & value);

  /*! \brief Insert an sva::ImpedanceVecd element into an array
   *
   * \see push(bool)
   */
  void push(const sva::ImpedanceVecd & value);

  /*! \brief Push a Configuration element into an array
   *
   * \see push(bool);
   */
  void push(const Configuration & value);

  /*! \brief Push an empty array
   *
   * \see push(bool);
   */
  Configuration array(size_t reserve = 0);

  /*! \brief Push an empty object
   *
   * \see push(bool);
   */
  Configuration object();

  /*! \brief User-defined conversion
   *
   * Requires the existence of:
   * mc_rtc::Configuration mc_rtc::ConfigurationLoader<T>::save(const T&, Args ...);
   *
   * \param key Key of the element
   *
   * \param value Value to push
   */
  template<typename T,
           typename... Args,
           typename std::enable_if<internal::has_configuration_save_object<T, Args...>::value, int>::type = 0>
  void push(const T & value, Args &&... args)
  {
    push(mc_rtc::ConfigurationLoader<T>::save(value, std::forward<Args>(args)...));
  }

  /*! \brief Add a vector into the JSON document
   *
   * Overwrites existing content if any.
   *
   * \param key Key of the element
   *
   * \param value Vector of elements to add
   */
  template<typename T, typename A = std::allocator<T>, typename... Args>
  void add(const std::string & key, const std::vector<T, A> & value, Args &&... args)
  {
    Configuration v = array(key, value.size());
    for(const auto & vi : value)
    {
      v.push(vi, std::forward<Args>(args)...);
    }
  }

  /*! \brief Add an array into the JSON document
   *
   * Overwrites existing content if any.
   *
   * \param key Key of the element
   *
   * \param value Array of elements to add
   */
  template<typename T, std::size_t N, typename... Args>
  void add(const std::string & key, const std::array<T, N> & value, Args &&... args)
  {
    Configuration v = array(key, N);
    for(const auto & vi : value)
    {
      v.push(vi, std::forward<Args>(args)...);
    }
  }

  /*! \brief Add a pair into the JSON document
   *
   * Overwrite existing content if any.
   *
   * \param key Key of the element
   *
   * \param value Pair to add
   */
  template<typename T1, typename T2, typename... Args>
  void add(const std::string & key, const std::pair<T1, T2> & value, Args &&... args)
  {
    Configuration v = array(key, 2);
    v.push(value.first, std::forward<Args>(args)...);
    v.push(value.second, std::forward<Args>(args)...);
  }

  /*! \brief Add string-indexed map into the JSON document
   *
   * Overwrites existing content if any.
   *
   * \param key Key of the element
   *
   * \param value Map of elements to add
   */
  template<typename T,
           class C = std::less<std::string>,
           class A = std::allocator<std::pair<const std::string, T>>,
           typename... Args>
  void add(const std::string & key, const std::map<std::string, T, C, A> & value, Args &&... args)
  {
    Configuration v = add(key);
    for(const auto & el : value)
    {
      v.add(el.first, el.second, std::forward<Args>(args)...);
    }
  }

  /*! \brief Add a set into the JSON document
   *
   * Overwrites existing content if any.
   *
   * \param key Key of the element
   *
   * \param value Set of elements to add
   */
  template<typename T, typename C = std::less<T>, typename A = std::allocator<T>, typename... Args>
  void add(const std::string & key, const std::set<T, C, A> & value, Args &&... args)
  {
    Configuration v = array(key, value.size());
    for(const auto & v : value)
    {
      v.push(*v, std::forward<Args>(args)...);
    }
  }

  /*! \brief Add an unordered set into the JSON document
   *
   * Overwrites existing content if any.
   *
   * \param key Key of the element
   *
   * \param value Set of elements to add
   */
  template<typename T,
           typename H = std::hash<T>,
           typename E = std::equal_to<T>,
           typename A = std::allocator<T>,
           typename... Args>
  void add(const std::string & key, const std::unordered_set<T, H, E, A> & value, Args &&... args)
  {
    Configuration v = array(key, value.size());
    for(const auto & v : value)
    {
      v.push(*v, std::forward<Args>(args)...);
    }
  }

  /*! \brief User-defined conversion
   *
   * Requires the existence of:
   * mc_rtc::Configuration mc_rtc::ConfigurationLoader<T>::save(const T&, Args ... args);
   *
   * \param key Key of the element
   *
   * \param value Value to add
   */
  template<typename T,
           typename... Args,
           typename std::enable_if<internal::has_configuration_save_object<T, Args...>::value, int>::type = 0>
  void add(const std::string & key, const T & value, Args &&... args)
  {
    add(key, ConfigurationLoader<T>::save(value, std::forward<Args>(args)...));
  }

  /*! \brief Push a vector into the JSON document
   *
   * \param value Vector of elements to add
   */
  template<typename T, typename A = std::allocator<T>, typename... Args>
  void push(const std::vector<T, A> & value, Args &&... args)
  {
    Configuration v = array(value.size());
    for(const auto & vi : value)
    {
      v.push(vi, std::forward<Args>(args)...);
    }
  }

  /*! \brief Push an array into the JSON document
   *
   * \param value Array of elements to add
   */
  template<typename T, std::size_t N, typename... Args>
  void push(const std::array<T, N> & value, Args &&... args)
  {
    Configuration v = array(N);
    for(const auto & vi : value)
    {
      v.push(vi, std::forward<Args>(args)...);
    }
  }

  /*! \brief Push a pair into the JSON document
   *
   * \param value Pair of elements to add
   */
  template<typename T1, typename T2, typename... Args>
  void push(const std::pair<T1, T2> & value, Args &&... args)
  {
    Configuration v = array(2);
    v.push(value.first, std::forward<Args>(args)...);
    v.push(value.second, std::forward<Args>(args)...);
  }

  /*! \brief Push a string-indexed map into the JSON document
   *
   * \param value Map of elements to add
   */
  template<typename T,
           class C = std::less<std::string>,
           class A = std::allocator<std::pair<const std::string, T>>,
           typename... Args>
  void push(const std::map<std::string, T, C, A> & value, Args &&... args)
  {
    Configuration v = object();
    for(const auto & el : value)
    {
      v.add(el.first, el.second, std::forward<Args>(args)...);
    }
  }

  /*! \brief Push a set into the JSON document
   *
   * Overwrites existing content if any.
   *
   * \param key Key of the element
   *
   * \param value Set of elements to add
   */
  template<typename T, typename C = std::less<T>, typename A = std::allocator<T>, typename... Args>
  void push(const std::set<T, C, A> & value, Args &&... args)
  {
    Configuration v = array(value.size());
    for(const auto & v : value)
    {
      v.push(*v, std::forward<Args>(args)...);
    }
  }

  /*! \brief Push an unordered set into the JSON document
   *
   * Overwrites existing content if any.
   *
   * \param key Key of the element
   *
   * \param value Set of elements to add
   */
  template<typename T,
           typename H = std::hash<T>,
           typename E = std::equal_to<T>,
           typename A = std::allocator<T>,
           typename... Args>
  void push(const std::unordered_set<T, H, E, A> & value, Args &&... args)
  {
    Configuration v = array(value.size());
    for(const auto & v : value)
    {
      v.push(*v, std::forward<Args>(args)...);
    }
  }

  /** Remove a given element
   *
   *  Has no effect if the element is not present.
   *
   *  \param key Element to remove
   *
   *  \returns True if the element was removed, false otherwise.
   *
   */
  bool remove(const std::string & key);

  /** Returns the keys of the JSON objects
   *
   * Returns an empty vector if the underlying JSON object is not an object
   */
  std::vector<std::string> keys() const;

  ConfigurationArrayIterator begin() const;

  ConfigurationArrayIterator end() const;

private:
  Json v;
  Configuration(const Json & v);
};

struct MC_RTC_UTILS_DLLAPI ConfigurationArrayIterator
{
  ConfigurationArrayIterator(const Configuration & conf);
  bool operator!=(const ConfigurationArrayIterator & rhs) const;
  ConfigurationArrayIterator & operator++();
  Configuration operator*();
  const Configuration operator*() const;
  size_t i = 0;
  Configuration conf;
};

/*! \brief Specialized version to lift ambiguity */
template<>
void MC_RTC_UTILS_DLLAPI Configuration::operator()(const std::string & key, std::string & v) const;

} // namespace mc_rtc

/*! \brief Ostream operator */
MC_RTC_UTILS_DLLAPI std::ostream & operator<<(std::ostream & os, const mc_rtc::Configuration & c);

namespace fmt
{

template<>
struct formatter<mc_rtc::Configuration> : public formatter<string_view>
{
  template<typename FormatContext>
  auto format(const mc_rtc::Configuration & c, FormatContext & ctx) -> decltype(ctx.out())
  {
    return formatter<string_view>::format(static_cast<std::string>(c), ctx);
  }
};

} // namespace fmt
