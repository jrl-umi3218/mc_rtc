#include <mc_rtc/logging.h>

#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace mc_rtc
{

namespace log
{

namespace details
{

spdlog::logger & success()
{
  static auto success = []() {
    auto success = spdlog::create_async_nb<spdlog::sinks::stdout_color_sink_mt>("success");
    success->set_pattern("%^[success]%$ %v");
    auto sink = static_cast<spdlog::sinks::stdout_color_sink_mt *>(success->sinks().back().get());
#ifndef WIN32
    sink->set_color(spdlog::level::info, "\033[01;32m"); // bold green
#else
    sink->set_color(spdlog::level::info, FOREGROUND_INTENSITY | FOREGROUND_GREEN);
#endif
    return success;
  }();
  return *success;
}

spdlog::logger & info()
{
  static auto info = []() {
    auto info = spdlog::create_async_nb<spdlog::sinks::stdout_color_sink_mt>("info");
    info->set_pattern("%^[info]%$ %v");
    auto sink = static_cast<spdlog::sinks::stdout_color_sink_mt *>(info->sinks().back().get());
#ifndef WIN32
    sink->set_color(spdlog::level::info, "\033[01;34m"); // bold cyan
#else
    sink->set_color(spdlog::level::info, FOREGROUND_INTENSITY | FOREGROUND_GREEN | FOREGROUND_BLUE);
#endif
    return info;
  }();
  return *info;
}

spdlog::logger & cerr()
{
  static auto cerr = []() {
    auto cerr = spdlog::create_async_nb<spdlog::sinks::stderr_color_sink_mt>("cerr");
    cerr->set_pattern("%^[%l]%$ %v");
    return cerr;
  }();
  return *cerr;
}

} // namespace details

} // namespace log

} // namespace mc_rtc
