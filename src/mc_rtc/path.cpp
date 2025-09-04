#include <mc_rtc/path.h>
#include <fmt/format.h>

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

std::string make_temporary_path(const std::string & prefix)
{
  auto tmp = bfs::temp_directory_path();
  auto pattern = fmt::format("{}-%%%%-%%%%-%%%%-%%%%", prefix);
  // std::filesystem does not have a unique_path function in c++17
  // keep boost around for now
  auto out = tmp / bfs::unique_path(pattern).string();
  bfs::create_directories(out);
  return out.string();
}

} // namespace mc_rtc
