#include <mc_rtc/logging.h>

#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#ifdef MC_RTC_HAS_LIBNOTIFY
#  include <libnotify/notification.h>
#  include <libnotify/notify.h>
#endif

#ifdef MC_RTC_HAS_WINTOAST
#  include <mc_rtc/version.h>

#  include <codecvt>
#  include <locale>

#  include "wintoastlib.h"
#endif

#ifdef MC_RTC_HAS_WSL_NOTIFY_SEND
#  include <internals/logging.h>
#  include <stdlib.h>
#endif

namespace mc_rtc
{

namespace log
{

namespace details
{

spdlog::logger & success()
{
  static auto success = []()
  {
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
  static auto info = []()
  {
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
  static auto cerr = []()
  {
    auto cerr = spdlog::create_async_nb<spdlog::sinks::stderr_color_sink_mt>("cerr");
    cerr->set_pattern("%^[%l]%$ %v");
    return cerr;
  }();
  return *cerr;
}

#ifdef MC_RTC_HAS_LIBNOTIFY

static bool do_notify_init = []()
{
  bool r = notify_init("mc_rtc");
  if(!r) { mc_rtc::log::warning("[mc_rtc] libnotify init failed"); }
  return r;
}();

static inline void do_notify(const std::string & message)
{
  if(!do_notify_init) { return; }
  auto n = notify_notification_new("[mc_rtc]", message.c_str(), nullptr);
  notify_notification_set_timeout(n, NOTIFY_EXPIRES_DEFAULT);
  notify_notification_set_urgency(n, NOTIFY_URGENCY_NORMAL);
  notify_notification_show(n, nullptr);
  g_object_unref(n);
}

#endif

#ifdef MC_RTC_HAS_WINTOAST

static bool do_notify_init()
{
  static bool initialized = []()
  {
    if(!WinToastLib::WinToast::isCompatible())
    {
      mc_rtc::log::warning("[mc_rtc] Failed to initialize WinToast");
      return false;
    }
    auto toaster = WinToastLib::WinToast::instance();
    toaster->setAppName(L"mc-rtc");
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    const auto aumi = WinToastLib::WinToast::configureAUMI(L"mc-rtc", L"mc-rtc");
    toaster->setAppUserModelId(aumi);
    if(!toaster->initialize())
    {
      mc_rtc::log::warning("[mc_rtc] Failed to initialize toaster");
      return false;
    }
    return true;
  }();
  return initialized;
}

struct ToastHandler : public WinToastLib::IWinToastHandler
{
  void toastActivated() const override {}
  void toastActivated(int) const override {}
  void toastDismissed(WinToastDismissalReason) const override {}
  void toastFailed() const override {}
};

static inline void do_notify(const std::string & message)
{
  if(!do_notify_init()) { return; }
  auto handler = new ToastHandler();
  auto tmpl = WinToastLib::WinToastTemplate(WinToastLib::WinToastTemplate::Text02);
  tmpl.setTextField(L"[mc_rtc]", WinToastLib::WinToastTemplate::FirstLine);
  std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
  tmpl.setTextField(converter.from_bytes(message), WinToastLib::WinToastTemplate::SecondLine);
  auto toaster = WinToastLib::WinToast::instance();
  if(!toaster->showToast(tmpl, handler)) { mc_rtc::log::warning("[mc_rtc] WinToast failed to show notification"); }
}

#endif

#ifdef MC_RTC_HAS_WSL_NOTIFY_SEND

static inline void do_notify(const std::string & message)
{
  auto escape_quote = [](std::string & msg, size_t pos)
  {
    auto xpos = msg.find('"', pos);
    if(xpos == std::string::npos) { return xpos; }
    msg.replace(xpos, 1, "\\\"");
    return xpos + 2;
  };
  size_t pos = 0;
  std::string msg = message;
  do { pos = escape_quote(msg, pos); } while(pos != std::string::npos);
  auto cmd = fmt::format("{} -c [mc_rtc] \"{}\"", MC_RTC_WSL_NOTIFY_SEND, msg);
  int ret = system(cmd.c_str());
  if(ret != 0) { mc_rtc::log::warning("[mc_rtc] Notification failed"); }
}

#endif

static bool NOTIFICATIONS_ENABLED = true;

void disable_notifications()
{
  NOTIFICATIONS_ENABLED = false;
}

void notify(const std::string & message)
{
  if(NOTIFICATIONS_ENABLED) { do_notify(message); }
}

} // namespace details

} // namespace log

} // namespace mc_rtc
