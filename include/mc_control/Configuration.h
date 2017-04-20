#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/Core>

#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <mc_control/api.h>

namespace mc_control
{

  /*! \brief Simplify access to values hold within a JSON file
   *
   * Configuration values can be accessed using a key system, type
   * conversion are ensured by the class.
   */
  struct MC_CONTROL_DLLAPI Configuration
  {

    /*! \brief Exception thrown by this class when something bad occurs
     */
    struct MC_CONTROL_DLLAPI Exception : public std::exception
    {
      /*! \brief Constructor
       *
       * \param msg The message that will be returned by what()
       */
      Exception(const std::string & msg);

      virtual const char * what() const noexcept override;

      std::string msg;
    };

    /*! \brief Check if the key is part of the conf
     *
     * \param key The key to test
     *
     * \returns True if key is part of the configuration
     */
    bool isMember(const std::string & key) const;

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
  private:

    /*! \brief Implementation details
     *
     * This structure is meant to hide the JSON library used by mc_rtc
     * while allowing the template method implementation.
     *
     */
    struct MC_CONTROL_DLLAPI Json
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

  /*! \brief Specialized version to lift ambiguity */
  template<>
  void MC_CONTROL_DLLAPI Configuration::operator()(const std::string & key, std::string & v) const;
}
