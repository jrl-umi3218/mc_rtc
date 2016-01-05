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
    bfs::directory_iterator dit(path), endit;
    auto drange = boost::make_iterator_range(dit, endit);
    for(const auto & p : drange)
    {
      /* Attempt to load anything that is not a directory */
      if(!bfs::is_directory(p))
      {
        lt_dlhandle h = lt_dlopen(p.path().string().c_str());
        if(h == nullptr)
        {
          LOG_WARNING("Failed to load " << p.path().string() << std::endl << lt_dlerror())
          continue;
        }
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wpedantic"
        const char * CLASS_NAME = ((const char *(*)(void))(lt_dlsym(h, "CLASS_NAME")))();
        #pragma GCC diagnostic pop
        if(CLASS_NAME == nullptr)
        {
          LOG_WARNING("No symbol CLASS_NAME in library " << p.path().string() << std::endl << lt_dlerror())
          continue;
        }
        std::string class_name(CLASS_NAME);
        if(out.count(class_name))
        {
          /* We get the first library that declared this class name and only
           * emit an exception if this is declared in a different file */
          bfs::path orig_p(lt_dlgetinfo(out[class_name])->filename);
          if(orig_p != p.path())
          {
            LOG_ERROR("Multiple files export the same name " << class_name << " (new declaration in " << p.path().string() << ", previous declaration in " << lt_dlgetinfo(out[class_name])->filename << ")")
            throw(LoaderException("Multiple libraries expose the same class"));
          }
        }
        out[class_name] = h;
      }
    }
  }
}

} // namespace mc_rtc
