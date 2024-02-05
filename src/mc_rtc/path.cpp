#include <mc_rtc/path.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_rtc
{

std::string temp_directory_path(const std::string & suffix)
{
  return (bfs::temp_directory_path() / suffix).string();
}

std::string user_config_directory_path(const std::string & suffix)
{
#ifndef WIN32
  return (bfs::path(std::getenv("HOME")) / ".config/mc_rtc" / suffix).string();
#else
  return (bfs::path(std::getenv("APPDATA")) / "mc_rtc" / suffix).string();
#endif
}

} // namespace mc_rtc
