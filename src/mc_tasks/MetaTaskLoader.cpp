#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/logging.h>

namespace mc_tasks
{

std::unique_ptr<std::map<std::string, MetaTaskLoader::load_fun>> MetaTaskLoader::fns_ptr;

MetaTaskPtr MetaTaskLoader::load(const mc_rbdyn::Robots & robots,
                                 const std::string & file)
{
  return load(robots, mc_rtc::Configuration(file));
}

MetaTaskPtr MetaTaskLoader::load(const mc_rbdyn::Robots & robots,
                                 const mc_rtc::Configuration & config)
{
  static auto & fns = get_fns();
  if(config.has("type"))
  {
    std::string type = config("type");
    if(fns.count(type))
    {
      return fns[type](robots, config);
    }
    LOG_ERROR_AND_THROW(std::runtime_error, "MetaTaskLoader cannot handle MetaTask type " << type)
  }
  LOG_ERROR_AND_THROW(std::runtime_error, "Attempted to load a MetaTask object without a type property")
}

bool MetaTaskLoader::register_load_function(const std::string & type,
                                            load_fun fn)
{
  static auto & fns = get_fns();
  if(fns.count(type) == 0)
  {
    fns[type] = fn;
    LOG_SUCCESS("Register MetaTaskLoader for " << type)
    return true;
  }
  LOG_ERROR_AND_THROW(std::runtime_error, type << " is already handled by another loading function")
}

std::map<std::string, MetaTaskLoader::load_fun> & MetaTaskLoader::get_fns()
{
  if(!fns_ptr)
  {
    fns_ptr = std::unique_ptr<std::map<std::string, load_fun>>(new std::map<std::string, load_fun>());
  }
  return *fns_ptr;
}

}
