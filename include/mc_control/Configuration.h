#pragma once

#include <json/json.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/Core>

#include <exception>
#include <string>
#include <vector>

namespace mc_control
{

  /*! \brief Simplify access to values hold within a Json::Value
   *
   * Configuration values can be accessed using a key system, type
   * conversion are ensured by the class.
   */
  struct Configuration
  {

    /*! \brief Exception thrown by this class when something bad occurs
     */
    struct Exception : public std::exception
    {
      /*! \brief Constructor
       *
       * \param msg The message that will be returned by what()
       */
      Exception(const std::string & msg);

      virtual const char * what() const noexcept override;

      std::string msg;
    };

    /*! \brief Represent entries in the configuration file
     *
     * These values are meant to be cast into useful types.
     */
    struct Entry
    {
      /*! \brief Constructor
       *
       * \param v Json::Value corresponding to the entry
       */
      Entry(Json::Value & v);

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
            ret.push_back(Entry(v[static_cast<int>(i)]));
          }
          return ret;
        }
        else
        {
          throw Configuration::Exception("Stored Json value is not a vector");
        }
      }

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
      Entry operator()(const std::string & key);

      /*! \brief Retrieve and store a given value stored within the
       * configuration
       *
       * If they key is not stored in the Configuration, the value is
       * unchanged.
       *
       * \param key The key used to store the value
       *
       * \param v The value to retriev
       *
       */
      template<typename T>
      void operator()(const std::string & key, T & v)
      {
        try
        {
          v = (*this)(key);
        }
        catch(Exception &)
        {
        }
      }

      Json::Value & v;
    };

    /*! \brief Constructor using an existing Json::Value */
    Configuration(const Json::Value & v);

    /*! \brief Constructor using a file path */
    Configuration(const std::string & path);

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
    Entry operator()(const std::string & key);

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
    void operator()(const std::string & key, T & v)
    {
      try
      {
        v = (*this)(key);
      }
      catch(Exception &)
      {
      }
    }
    /*! \brief Holds the full configuration as Json::Value */
    Json::Value v;
  };

  template<>
  void Configuration::operator()(const std::string & key, std::string & v)
  {
    try
    {
      v = (std::string)(*this)(key);
    }
    catch(Exception &)
    {
    }
  }

  template<>
  void Configuration::Entry::operator()(const std::string & key, std::string & v)
  {
    try
    {
      v = (std::string)(*this)(key);
    }
    catch(Exception &)
    {
    }
  }
}
