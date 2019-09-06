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

namespace mc_rtc
{

LTDLHandle::~LTDLHandle()
{
  close();
}

LTDLHandle::LTDLHandle(const std::string & class_name, const std::string & path, bool verbose)
: path_(path), verbose_(verbose)
{
  auto get_classes = get_symbol<void (*)(std::vector<std::string> &)>(class_name);
  valid_ = get_classes != nullptr;
  if(valid_)
  {
    if(verbose_)
    {
      LOG_INFO("Found matching class name symbol " << class_name)
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
    LOG_INFO("Attempt to open " << path_)
  }
#ifndef WIN32
  if(global_)
  {
    if(verbose_)
    {
      LOG_INFO("Opening " << path_ << " in global mode")
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
      LOG_WARNING("Failed to load " << path_ << "\n" << error)
    }
  }
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
      LOG_ERROR_AND_THROW(LoaderException, "Failed to initialize ltdl" << std::endl << error)
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
      LOG_ERROR_AND_THROW(LoaderException, "Failed to close ltdl" << std::endl << error)
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
  for(const auto & path : paths)
  {
    if(!bfs::exists(path))
    {
      if(verbose)
      {
        LOG_WARNING("Tried to load libraries from " << path << " which does not exist")
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
        auto handle = std::make_shared<LTDLHandle>(class_name, p.string(), verbose);
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
                LOG_WARNING("Multiple files export the same name " << cn << " (new declaration in " << p.string()
                                                                   << ", previous declaration in " << out[cn]->path()
                                                                   << ")")
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
