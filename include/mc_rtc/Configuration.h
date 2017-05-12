#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/Core>

#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <mc_rtc/utils_api.h>

#include <mc/rtc/deprecated.hh>

namespace mc_rtc
{

  struct MC_RTC_UTILS_DLLAPI ConfigurationArrayIterator;

  /*! \brief Simplify access to values hold within a JSON file
   *
   * Configuration values can be accessed using a key system, type
   * conversion are ensured by the class.
   */
  struct MC_RTC_UTILS_DLLAPI Configuration
  {

    /*! \brief Exception thrown by this class when something bad occurs
     */
    struct MC_RTC_UTILS_DLLAPI Exception : public std::exception
    {
      /*! \brief Constructor
       *
       * \param msg The message that will be returned by what()
       */
      Exception(const std::string & msg);

      virtual const char * what() const noexcept override;

      std::string msg;
    };

    /*! \brief Deprecated, see has
     *
     * \param key The key to test
     *
     * \returns True if key is part of the configuration
     */
    bool isMember(const std::string & key) const MC_RTC_DEPRECATED;

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
          ret.push_back(Configuration(v[static_cast<int>(i)]));
        }
        return ret;
      }
      else
      {
        throw Configuration::Exception("Stored Json value is not a vector");
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
        throw Configuration::Exception("Stored Json value is not an array of size 2");
      }
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

    /*! \brief Load more data into the configuration
     *
     * \param path Path to the configuration file
     *
     */
    void load(const std::string & path);

    /*! \brief Save the configuration to a file.
     *
     * \param path Path to the configuration file
     *
     * \param pretty Writes a human-readable file, defaults to true
     *
     */
    void save(const std::string & path, bool pretty = true);

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

    /*! \brief Retrieve and store a given value stored within the
     * configuration
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
      catch(Exception &)
      {
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

    /*! \brief Add a double element to the Configuration
     *
     * \see add(const std::string&, bool)
     */
    void add(const std::string & key, double value);

    /*! \brief Add a std::string element to the Configuration
     *
     * \see add(const std::string&, bool)
     */
    void add(const std::string & key, std::string value);

    /*! \brief Add a Eigen::Vector3d element to the Configuration
     *
     * \see add(const std::string&, bool)
     */
    void add(const std::string & key, Eigen::Vector3d value);

    /*! \brief Add a Eigen::Vector6d element to the Configuration
     *
     * \see add(const std::string&, bool)
     */
    void add(const std::string & key, Eigen::Vector6d value);


    /*! \brief Add a Eigen::VectorXd element to the Configuration
     *
     * \see add(const std::string&, bool)
     */
    void add(const std::string & key, Eigen::VectorXd value);

    /*! \brief Add a Eigen::Quaterniond element to the Configuration
     *
     * \see add(const std::string&, bool)
     */
    void add(const std::string & key, Eigen::Quaterniond value);

    /*! \brief Add another Configuration to the Configuration
     *
     * \see add(const std::string&, bool)
     */
    void add(const std::string & key, Configuration value);

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

    /*! \brief Insert a double element into an array
     *
     * \see push(bool);
     */
    void push(double value);

    /*! \brief Insert a std::string element into an array
     *
     * \see push(bool);
     */
    void push(std::string value);

    /*! \brief Insert a Eigen::Vector3d element into an array
     *
     * \see push(bool);
     */
    void push(Eigen::Vector3d value);

    /*! \brief Insert a Eigen::Vector6d element into an array
     *
     * \see push(bool);
     */
    void push(Eigen::Vector6d value);

    /*! \brief Insert a Eigen::VectorXd element into an array
     *
     * \see push(bool);
     */
    void push(Eigen::VectorXd value);

    /*! \brief Insert a Eigen::Quaterniond element into an array
     *
     * \see push(bool);
     */
    void push(Eigen::Quaterniond value);

    /*! \brief Push a Configuration element into an array
     *
     * \see push(bool);
     */
    void push(Configuration value);

    /*! \brief Add a vector into the JSON document
     *
     * Overwrites existing content if any.
     *
     * \param key Key of the element
     *
     * \param value Vector of elements to add
     */
    template<typename T, typename A>
    void add(const std::string & key, const std::vector<T, A> & value)
    {
      Configuration v = array(key);
      for(const auto & vi : value)
      {
        v.push(vi);
      }
    }

    /*! \brief Push a vector into the JSON document
     *
     * \param value Vector of elements to add
     */
    template<typename T, typename A>
    void push(const std::vector<T, A> & value)
    {
      Configuration v = array(value.size());
      for(const auto & vi : value)
      {
        v.push(vi);
      }
    }

    ConfigurationArrayIterator begin() const;

    ConfigurationArrayIterator end() const;
  private:
    /*! \brief Create an empty array */
    Configuration array(size_t reserve);

    /*! \brief Implementation details
     *
     * This structure is meant to hide the JSON library used by mc_rtc
     * while allowing the template method implementation.
     *
     */
    struct MC_RTC_UTILS_DLLAPI Json
    {
      struct Impl;
      bool isArray() const;
      size_t size() const;
      Json operator[](int idx) const;
      Json operator[](const std::string & key) const;
      std::shared_ptr<Impl> impl;
    };
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

}

/*! \brief Ostream operator */
MC_RTC_UTILS_DLLAPI std::ostream & operator<<(std::ostream & os, const mc_rtc::Configuration & c);
