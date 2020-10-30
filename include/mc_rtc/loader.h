/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/*! \file loader.h
 * \brief ltdl wrapper for mc_rtc purpose
 *
 * The purpose of this file is to wrap ltdl
 * functionnalities in a convenient fashion for usage in both mc_robots and
 * mc_control The assumption is that at least the CLASS_NAME symbol is exported
 * in the libraries that should be loaded through this interface
 */

#include <mc_rtc/DataStore.h>
#include <mc_rtc/loader_api.h>
#include <mc_rtc/logging.h>

#include <boost/noncopyable.hpp>

#ifndef MC_RTC_BUILD_STATIC
#  include <ltdl.h>
#endif

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mc_rtc
{

template<typename T>
struct ObjectLoader;

/*! \class LoaderException
 * \brief Exception thrown by loader interface
 */
struct MC_RTC_LOADER_DLLAPI LoaderException : public std::exception
{
public:
  LoaderException(const std::string & what) : what_(what) {}

  virtual const char * what() const noexcept override
  {
    return what_.c_str();
  }

private:
  std::string what_;
};

/*! \class LTDLHandle
 *
 * \brief Wrapper around lt_dlhandle
 *
 * On creation it will load the library to check:
 * - if a provided symbol is defined in the library
 * - if it should be loaded globally
 *
 * After that the library will be closed and opened when symbols are requested by the user
 */
struct MC_RTC_LOADER_DLLAPI LTDLHandle
{
  /** Create the handle wrapper
   *
   * \param class_name Symbol that should be checked for
   *
   * \param path Path to the library
   *
   * \param rpath Path(s) used to search for libraries, this is only used on Windows,
   *        on Linux/macOS one is advised to use RPATH capabilties
   *
   * \param verbose If true, output debug information
   */
  LTDLHandle(const std::string & class_name, const std::string & path, const std::string & rpath, bool verbose);

  ~LTDLHandle();

  LTDLHandle(const LTDLHandle &) = delete;
  LTDLHandle & operator=(const LTDLHandle &) = delete;

  /** Get a symbol, returns nullptr if the symbol is not found
   *
   * \param name Name of the symbol to get
   */
  template<typename SymT>
  SymT get_symbol(const std::string & name);

  /** True if the library is valid */
  inline bool valid() const
  {
    return valid_;
  }

  /** Returns a list of available classes provided by this library */
  inline const std::vector<std::string> & classes() const
  {
    return classes_;
  }

  /** Access the path to the library */
  inline const std::string & path() const
  {
    return path_;
  }

private:
  std::string path_;
  std::string rpath_;
  bool verbose_;
#ifndef MC_RTC_BUILD_STATIC
  lt_dlhandle handle_;
#endif
  bool valid_ = false;
  bool global_ = false;
  bool open_ = false;
  std::vector<std::string> classes_;

  bool open();
  void close();
};

using LTDLHandlePtr = std::shared_ptr<LTDLHandle>;

/*! \class Loader
 * \brief General wrapper for ltdl functionnalities
 */
struct MC_RTC_LOADER_DLLAPI Loader
{
  template<typename T>
  friend struct ObjectLoader;
  typedef std::map<std::string, LTDLHandlePtr> handle_map_t;
  typedef std::function<void(const std::string &, LTDLHandle &)> callback_t;

public:
  static callback_t default_cb;

  /** Suffix appended to libraries paths when running in debug mode */
  static std::string debug_suffix;

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
   *
   * \param class_name The loader will filter out libraries that do not
   * provide a class_name function that returns the object's name, this
   * allows to filter the loaded libraries
   *
   * \param paths a list of the directories searched by the function
   *
   * \param out a map (string, handle_type) updated by the function
   *
   * \param verbose If true, output some warning information
   *
   * \param cb User-provided callback when a new class is discovered
   *
   * \anchor loader_load_libraries_doc
   */
  static void load_libraries(const std::string & class_name,
                             const std::vector<std::string> & paths,
                             handle_map_t & out,
                             bool verbose,
                             callback_t cb);

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
   *
   * \param class_name Symbol used to distinguish the relevant libraries
   *
   * \param paths directories searched for libraries
   *
   * \param enable_sandbox If true, creation function called from the
   * loaded modules are sandboxed allowing to recover from otherwise
   * fatal crashes
   *
   * \param verbose If true, output some warning information
   *
   * \param cb Function called when a new object is loaded
   *
   */
  ObjectLoader(const std::string & class_name,
               const std::vector<std::string> & paths,
               bool enable_sandbox,
               bool verbose,
               Loader::callback_t cb = Loader::default_cb);

  /** Destructor */
  ~ObjectLoader();

  /** Returns true if the loader has the name object
   * \param name Name to be tested
   */
  bool has_object(const std::string & name) const;

  /** Returns the list of available objects
   * \return A list of available objects
   */
  std::vector<std::string> objects() const;

  /** Load libraries from the paths provided
   * \param paths directories searched for libraries
   * \param verbose If true, output some warning information
   */
  void load_libraries(const std::vector<std::string> & paths, Loader::callback_t cb = Loader::default_cb);

  /** Remove all loaded libraries */
  void clear();

  /** Enable/disable sandboxing
   *
   * \param enable_sandbox If true, object's creation is done in a
   * sandbox environment
   *
   */
  void enable_sandboxing(bool enable_sandbox);

  /** Set the verbosity
   *
   * \param verbose If true, loader will be more verbose
   *
   */
  void set_verbosity(bool verbose);

  /** Register a new loading function
   *
   * \param name Name that will be used to create new instance
   *
   * \param callback Callback that will be used to create the object
   *
   * \tparam RetT Must be derived from T
   *
   * \throws LoaderException if the object is already registered
   */
  template<typename RetT, typename... Args>
  void register_object(const std::string & name, std::function<RetT *(const Args &...)> callback);

  /** Create a new object of type name
   * \param name the object's name
   * \param Args argument required by the constructor
   * \return a shared pointer properly equipped to destroy the pointer
   * \throws LoaderException throw if the name does not exist or if symbol resolution fails
   */
  template<typename... Args>
  std::shared_ptr<T> create_object(const std::string & name, Args... args);

  struct ObjectDeleter
  {
    ObjectDeleter() {}
    ObjectDeleter(void (*fn)(T *));
    void operator()(T * ptr);

  private:
    void (*delete_fn_)(T *) = nullptr;
  };

  using unique_ptr = std::unique_ptr<T, ObjectDeleter>;

  /** Create a new object of type name
   *
   * \param name the object's name
   *
   * \param Args arguments required by the constructor
   *
   * \returns a unique pointer with a destructor provided by the library
   *
   * \throws LoaderException if the name does not exist or if symbol resolution fails
   */
  template<typename... Args>
  unique_ptr create_unique_object(const std::string & name, Args... args);

protected:
  std::string class_name;
  bool enable_sandbox;
  bool verbose;
  Loader::handle_map_t handles_;
  mc_rtc::DataStore callbacks_;
  std::unordered_map<std::string, ObjectDeleter> deleters_;

  /** Internal function to create a raw pointer */
  template<typename... Args>
  T * create(const std::string & name, Args... args);

  /** Internal function to create a raw pointer from an handle */
  template<typename... Args>
  T * create_from_handles(const std::string & name, Args... args);

  /** Internal function to create a raw pointer from a registered callback */
  template<typename... Args>
  T * create_from_callbacks(const std::string & name, Args... args);
};

} // namespace mc_rtc

#include <mc_rtc/loader.hpp>
