#pragma once

/*! \file loader.h
 * \brief ltdl wrapper for mc_rtc purpose
 *
 * The purpose of this file is to wrap ltdl
 * functionnalities in a convenient fashion for usage in both mc_robots and
 * mc_control The assumption is that at least the CLASS_NAME symbol is exported
 * in the libraries that should be loaded through this interface
*/

#include <mc_rtc/loader_api.h>
#include <mc_rtc/logging.h>

#include <boost/noncopyable.hpp>

#include <ltdl.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mc_rtc
{

template<typename T> struct ObjectLoader;

/*! \class LoaderException
 * \brief Exception thrown by loader interface
*/
struct MC_RTC_LOADER_DLLAPI LoaderException : public std::exception
{
public:
  LoaderException(const std::string & what) : what_(what) {}

  virtual const char * what() const noexcept override { return what_.c_str(); }
private:
  std::string what_;
};

/*! \class Loader
 * \brief General wrapper for ltdl functionnalities
*/
struct MC_RTC_LOADER_DLLAPI Loader
{
template<typename T> friend struct ObjectLoader;
typedef std::map<std::string, lt_dlhandle> handle_map_t;
protected:
  /*! \brief Initialize ltdl
   * \throws LoaderException if ltdl fails to init
  */
  static bool init();

  /*! \brief Close ltdl. Does nothing until the last call
   * \throws LoaderException on failure
  */
  static bool close();

  /*! \brief Provide libraries handles for the libraries in paths
   * \param paths a list of the directories searched by the function
   * \param out a map (string, handle_type) updated by the function
   * \throws LoaderException if multiple libraries return the same CLASS_NAME() or if the name is already in out
   * \anchor loader_load_libraries_doc
  */
  static void load_libraries(const std::vector<std::string> & paths, handle_map_t & out);
private:
  static unsigned int init_count_;
};

/*! \class ObjectLoader
 * \brief ltdl wrapper for factory-like classes
*/
template<typename T>
struct ObjectLoader : public boost::noncopyable
{
public:
  /** Create ObjectLoader instance
   * \param paths directories searched for libraries
   * \throws See \ref loader_load_libraries_doc "Loader load_libraries throwing condition"
  */
  ObjectLoader(const std::vector<std::string> & paths);

  /** Destructor */
  ~ObjectLoader();

  /** Returns true if the loader has the name object
   * \param name Name to be tested
  */
  bool has_object(const std::string & name);

  /** Returns true if the loader has the name object and the symbol symbol in this library
   * \param name Name to be tested
   * \param symbol Symbol to be tested
  */
  bool has_symbol(const std::string & name, const std::string & symbol);

  /** Returns the list of available objects
   * \return A list of available objects
   */
  std::vector<std::string> objects() const;

  /** Load libraries from the paths provided
   * \param paths directories searched for libraries
   * \throws See \ref loader_load_libraries_doc "Loader load_libraries throwing condition"
  */
  void load_libraries(const std::vector<std::string> & paths);

  /** Create a new object of type name
   * \param name the object's name
   * \param Args argument required by the constructor
   * \return a shared pointer properly equipped to destroy the pointer
   * \throws LoaderException throw if the name does not exist or if symbol resolution fails
  */
  template<typename... Args>
  std::shared_ptr<T> create_object(const std::string & name, const Args & ... args);
protected:
  Loader::handle_map_t handles_;
  struct ObjectDeleter
  {
    ObjectDeleter() {}
    ObjectDeleter(void * sym);
    void operator()(T * ptr);
    std::function<void(T*)> delete_fn_;
  };
  std::map<std::string, ObjectDeleter> deleters_;
};

} // namespace mc_rtc

#include <mc_rtc/loader.hpp>
