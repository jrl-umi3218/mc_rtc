/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

/* !!! WARNING !!!
 * loader.cpp is a generated file
 * if you wish to change this file, edit:
 * @CMAKE_CURRENT_SOURCE_DIR@/mc_rtc/loader.in.cpp
 */

#include <mc_rtc/loader.h>

#include <boost/filesystem.hpp>
#include <boost/range/adaptors.hpp>
namespace bfs = boost::filesystem;

#ifdef WIN32

#  include <Windows.h>

namespace
{

/** Get the PATH variable at the program start */
char * getPATH()
{
  static std::unique_ptr<char> PATH;
  if(PATH)
  {
    return PATH.get();
  }
  int plen = GetEnvironmentVariable("PATH", nullptr, 0);
  PATH.reset(new char[plen]);
  GetEnvironmentVariable("PATH", PATH.get(), plen);
  return PATH.get();
};

} // namespace
#endif

namespace mc_rtc
{

LTDLHandle::~LTDLHandle()
{
  close();
}

LTDLHandle::LTDLHandle(const std::string & class_name,
                       const std::string & path,
                       const std::string & rpath,
                       bool verbose)
: path_(path), rpath_(rpath), verbose_(verbose)
{
  auto get_classes = get_symbol<void (*)(std::vector<std::string> &)>(class_name);
  valid_ = get_classes != nullptr;
  if(valid_)
  {
    if(verbose_)
    {
      mc_rtc::log::info("Found matching class name symbol {}", class_name);
    }
    get_classes(classes_);
  }
  if(valid_)
  {
    global_ = get_symbol<void (*)()>("LOAD_GLOBAL") != nullptr;
  }
  close();
}

bool LTDLHandle::open()
{
  if(open_)
  {
    return true;
  }
  if(verbose_)
  {
    mc_rtc::log::info("Attempt to open {}", path_);
#ifdef WIN32
    mc_rtc::log::info("Search path: {}", rpath_);
#endif
  }
#ifdef WIN32
  SetEnvironmentVariable("PATH", rpath_.c_str());
#endif
#ifndef WIN32
  if(global_)
  {
    if(verbose_)
    {
      mc_rtc::log::info("Opening {} in global mode", path_);
    }
    lt_dladvise advise;
    lt_dladvise_init(&advise);
    lt_dladvise_global(&advise);
    handle_ = lt_dlopenadvise(path_.c_str(), advise);
    lt_dladvise_destroy(&advise);
  }
  else
  {
    handle_ = lt_dlopen(path_.c_str());
  }
#else
  handle_ = lt_dlopen(path_.c_str());
#endif
  open_ = handle_ != nullptr;
  if(!open_)
  {
    const char * error = lt_dlerror();
    /* Discard the "file not found" error as it only indicates that we tried to load something other than a library */
    if(strcmp(error, "file not found") != 0)
    {
      mc_rtc::log::warning("Failed to load {}\n{}", path_, error);
    }
  }
#ifdef WIN32
  SetEnvironmentVariable("PATH", getPATH());
#endif
  return open_;
}

void LTDLHandle::close()
{
  if(open_)
  {
    open_ = false;
    lt_dlclose(handle_);
  }
}

Loader::callback_t Loader::default_cb = [](const std::string &, LTDLHandle &) {};

unsigned int Loader::init_count_ = 0;

bool Loader::init()
{
  if(init_count_ == 0)
  {
    int err = lt_dlinit();
    if(err != 0)
    {
      std::string error = lt_dlerror();
      mc_rtc::log::error_and_throw<LoaderException>("Failed to initialize ltdl\n{}", error);
    }
  }
  ++init_count_;
  return true;
}

bool Loader::close()
{
  --init_count_;
  if(init_count_ == 0)
  {
    int err = lt_dlexit();
    if(err != 0)
    {
      std::string error = lt_dlerror();
      mc_rtc::log::error_and_throw<LoaderException>("Failed to close ltdl\n{}", error);
    }
  }
  return true;
}

void Loader::load_libraries(const std::string & class_name,
                            const std::vector<std::string> & paths,
                            Loader::handle_map_t & out,
                            bool verbose,
                            Loader::callback_t cb)
{
#ifdef WIN32
  std::stringstream ss;
  for(const auto & path : paths)
  {
    ss << path << ";";
  }
  ss << getPATH();
  std::string rpath = ss.str();
#else
  std::string rpath = "";
#endif
  for(const auto & path : paths)
  {
    if(!bfs::exists(path))
    {
      if(verbose)
      {
        mc_rtc::log::warning("Tried to load libraries from {} which does not exist", path);
      }
      continue;
    }
    bfs::directory_iterator dit(path), endit;
    std::vector<bfs::path> drange;
    std::copy(dit, endit, std::back_inserter(drange));
    // Sort by newest file
    std::sort(drange.begin(), drange.end(), [](const bfs::path & p1, const bfs::path & p2) {
      return bfs::last_write_time(p1) > bfs::last_write_time(p2);
    });
    for(const auto & p : drange)
    {
      /* Attempt to load all dynamics libraries in the directory */
      if((!bfs::is_directory(p)) && (!bfs::is_symlink(p)) && bfs::extension(p) == "@CMAKE_SHARED_LIBRARY_SUFFIX@")
      {
        auto handle = std::make_shared<LTDLHandle>(class_name, p.string(), rpath, verbose);
        for(const auto & cn : handle->classes())
        {
          if(out.count(cn))
          {
            /* We get the first library that declared this class name and only
             * emit an exception if this is declared in a different file */
            bfs::path orig_p(out[cn]->path());
            if(orig_p != p)
            {
              if(verbose)
              {
                mc_rtc::log::warning(
                    "Multiple files export the same name {} (new declaration in {}, previous declaration in {})", cn,
                    p.string(), out[cn]->path());
              }
              continue;
            }
          }
          out[cn] = handle;
          cb(cn, *handle);
        }
      }
    }
  }
}

} // namespace mc_rtc
