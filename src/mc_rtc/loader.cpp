#include <mc_rtc/loader.h>

#include <boost/filesystem.hpp>
#include <boost/range/adaptors.hpp>
namespace bfs = boost::filesystem;

namespace mc_rtc
{

unsigned int Loader::init_count_ = 0;

bool Loader::init()
{
  if(init_count_ == 0)
  {
    int err = lt_dlinit();
    if(err != 0)
    {
      std::string error = lt_dlerror();
      LOG_ERROR("Failed to initialize ltdl" << std::endl << error)
      throw(LoaderException(error));
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
      LOG_ERROR("Failed to close ltdl" << std::endl << error)
      throw(LoaderException(error));
    }
  }
  return true;
}

void Loader::load_libraries(const std::vector<std::string> & paths, Loader::handle_map_t & out)
{
  for(const auto & path : paths)
  {
    if(!bfs::exists(path))
    {
      LOG_WARNING("Tried to load libraries from " << path << " which does not exist")
      continue;
    }
    bfs::directory_iterator dit(path), endit;
    std::vector<bfs::path> drange;
    std::copy(dit, endit, std::back_inserter(drange));
    // Sort by newest file
    std::sort(drange.begin(), drange.end(),
              [](const bfs::path & p1, const bfs::path & p2)
              {
                return bfs::last_write_time(p1) >
                       bfs::last_write_time(p2);
              }
             );
    for(const auto & p : drange)
    {
      /* Attempt to load anything that is not a directory */
      if(!bfs::is_directory(p))
      {
        lt_dlhandle h = lt_dlopen(p.string().c_str());
        if(h == nullptr)
        {
          /* Discard the "file not found" error as it only indicates that we
           * tried to load something other than a library */
          if(strcmp(lt_dlerror(), "file not found") != 0)
          {
            LOG_WARNING("Failed to load " << p.string() << std::endl << lt_dlerror())
          }
          continue;
        }
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wpedantic"
        typedef const char*(*class_name_fun_t)(void);
        class_name_fun_t CLASS_NAME_FUN = (class_name_fun_t)(lt_dlsym(h, "CLASS_NAME"));
        #pragma GCC diagnostic pop
        if(CLASS_NAME_FUN == nullptr)
        {
          LOG_WARNING("No symbol CLASS_NAME in library " << p.string() << std::endl << lt_dlerror())
          continue;
        }
        std::string class_name(CLASS_NAME_FUN());
        if(out.count(class_name))
        {
          /* We get the first library that declared this class name and only
           * emit an exception if this is declared in a different file */
          bfs::path orig_p(lt_dlgetinfo(out[class_name])->filename);
          if(orig_p != p)
          {
            LOG_WARNING("Multiple files export the same name " << class_name << " (new declaration in " << p.string() << ", previous declaration in " << lt_dlgetinfo(out[class_name])->filename << ")")
            continue;
          }
        }
        out[class_name] = h;
      }
    }
  }
}

} // namespace mc_rtc
