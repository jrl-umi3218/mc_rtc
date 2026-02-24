#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/path.h>
#include <fmt/format.h>

#include <boost/filesystem.hpp>
#include <filesystem>
#include <string_view>
namespace fs = std::filesystem;
namespace bfs = boost::filesystem;

namespace mc_rtc
{

std::string temp_directory_path(const std::string & suffix)
{
  return (fs::temp_directory_path() / suffix).string();
}

std::string user_config_directory_path(const std::string & suffix)
{
#ifndef WIN32
  return (fs::path(std::getenv("HOME")) / ".config/mc_rtc" / suffix).string();
#else
  return (fs::path(std::getenv("APPDATA")) / "mc_rtc" / suffix).string();
#endif
}

std::string local_share_directory(const std::string & suffix)
{
#ifdef WIN32
  return (fs::path(std::getenv("APPDATA")) / "mc_rtc" / suffix).string();
#else
  return (fs::path(std::getenv("HOME")) / ".local/share/mc_rtc" / suffix).string();
#endif
}

std::string filename(const fs::path & path, bool keepExtension)
{
  if(keepExtension)
    return path.filename().string();
  else
    return path.filename().stem().string();
}

fs::path convertURI(const std::string & uri, std::string_view default_dir)
{
  const std::string package = "package://";
  if(uri.size() >= package.size() && uri.find(package) == 0)
  {
    size_t split = uri.find('/', package.size());
    std::string pkg = uri.substr(package.size(), split - package.size());
    auto leaf = fs::path(uri.substr(split + 1));
    fs::path MC_ENV_DESCRIPTION_PATH(mc_rtc::MC_ENV_DESCRIPTION_PATH);
#ifndef __EMSCRIPTEN__
#  ifdef MC_RTC_HAS_ROS_SUPPORT
    try
    {
#    ifdef MC_RTC_ROS_IS_ROS2
      pkg = ament_index_cpp::get_package_share_directory(pkg);
#    else
      pkg = ros::package::getPath(pkg);
      if(pkg.empty()) { throw std::runtime_error("Package not found"); }
#    endif
      return pkg / leaf;
    }
    catch(...)
    {
      // ROS package not found or other error, fall through to fallback logic
    }
#  endif
    // Fallback for non-ROS builds or when ROS package is not found
    if(pkg == "jvrc_description") { pkg = mc_rtc::JVRC_DESCRIPTION_PATH; }
    else if(pkg == "mc_env_description") { pkg = mc_rtc::MC_ENV_DESCRIPTION_PATH; }
    else if(pkg == "mc_int_obj_description") { pkg = mc_rtc::MC_INT_OBJ_DESCRIPTION_PATH; }
    else
    { // could not resolve path using ament index, check for default_dir
      if(default_dir.empty())
      {
        mc_rtc::log::error_and_throw("[mc_rtc::convertURI] Could not resolve path to ROS package path package '{}' in "
                                     "URI '{}', and no default_dir was provided",
                                     pkg, uri);
        return uri;
      }
      else if(!fs::exists(default_dir) || !fs::is_directory(default_dir))
      {
        mc_rtc::log::error_and_throw("[mc_rtc::convertURI] Could not resolve path to ROS package path package '{}' in "
                                     "URI '{}', and default_dir '{}' does not exist or is not a directory",
                                     pkg, uri, default_dir);
        return uri;
      }
      else
      { // default_dir exists
        pkg = default_dir;
      }
    }
#else
    pkg = "/assets/" + pkg;
#endif
    return pkg / leaf;
  }
  const std::string file = "file://";
  if(uri.size() >= file.size() && uri.find(file) == 0) { return fs::path(uri.substr(file.size())); }
  return uri;
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
