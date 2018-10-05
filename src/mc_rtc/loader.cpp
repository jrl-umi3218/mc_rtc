#include <mc_rtc/loader.h>

#include <boost/filesystem.hpp>
#include <boost/range/adaptors.hpp>
namespace bfs = boost::filesystem;

namespace mc_rtc
{

Loader::callback_t Loader::default_cb = [](const std::string &, lt_dlhandle) {};

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
      /* Attempt to load anything that is not a directory */
      if((!bfs::is_directory(p)) && (!bfs::is_symlink(p)))
      {
        if(verbose)
        {
          LOG_INFO("Attempt to open " << p.string());
        }
        lt_dlhandle h = lt_dlopen(p.string().c_str());
        if(h == nullptr)
        {
          /* Discard the "file not found" error as it only indicates that we
           * tried to load something other than a library */
          const char * error = lt_dlerror();
          if(strcmp(error, "file not found") != 0)
          {
            if(verbose)
            {
              LOG_WARNING("Failed to load " << p.string() << std::endl << error)
            }
          }
          if(verbose)
          {
            LOG_WARNING("Skipping " << p.string() << std::endl << error)
          }
          continue;
        }
#ifndef WIN32
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wpedantic"
        typedef void (*load_global_fun_t)(void);
        load_global_fun_t LOAD_GLOBAL_FUN = (load_global_fun_t)(lt_dlsym(h, "LOAD_GLOBAL"));
#  pragma GCC diagnostic pop
        if(LOAD_GLOBAL_FUN != nullptr)
        {
          if(verbose)
          {
            LOG_INFO("Re-open " << p.string() << " in global mode")
          }
          lt_dlclose(h);
          lt_dladvise advise;
          lt_dladvise_init(&advise);
          lt_dladvise_global(&advise);
          h = lt_dlopenadvise(p.string().c_str(), advise);
          lt_dladvise_destroy(&advise);
        }
        else
        {
          /* Discard error message */
          lt_dlerror();
        }
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
        typedef void (*class_name_fun_t)(std::vector<std::string> &);
        class_name_fun_t CLASS_NAME_FUN = (class_name_fun_t)(lt_dlsym(h, class_name.c_str()));
#pragma GCC diagnostic pop
        if(CLASS_NAME_FUN == nullptr)
        {
          const char * error = lt_dlerror();
          if(verbose)
          {
            LOG_WARNING("No symbol " << class_name << " in library " << p.string() << std::endl << error)
          }
          continue;
        }
        if(verbose)
        {
          LOG_INFO("Found matching class name symbol " << class_name)
        }
        std::vector<std::string> class_names;
        CLASS_NAME_FUN(class_names);
        for(const auto & cn : class_names)
        {
          if(out.count(cn))
          {
            /* We get the first library that declared this class name and only
             * emit an exception if this is declared in a different file */
            bfs::path orig_p(lt_dlgetinfo(out[cn])->filename);
            if(orig_p != p)
            {
              if(verbose)
              {
                LOG_WARNING("Multiple files export the same name " << cn << " (new declaration in " << p.string()
                                                                   << ", previous declaration in "
                                                                   << lt_dlgetinfo(out[cn])->filename << ")")
              }
              continue;
            }
          }
          out[cn] = h;
          cb(cn, h);
        }
      }
    }
  }
}

} // namespace mc_rtc
