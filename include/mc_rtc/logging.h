/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/utils_api.h>

#include <mc_rtc/fmt_formatters.h>

#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/logger.h>

#define BOOST_STACKTRACE_LINK
#include <boost/stacktrace.hpp>

namespace mc_rtc
{

namespace log
{

namespace details
{

MC_RTC_UTILS_DLLAPI spdlog::logger & success();

MC_RTC_UTILS_DLLAPI spdlog::logger & info();

MC_RTC_UTILS_DLLAPI spdlog::logger & cerr();

MC_RTC_UTILS_DLLAPI void notify(const std::string & message);

MC_RTC_UTILS_DLLAPI void disable_notifications();

} // namespace details

template<typename ExceptionT = std::runtime_error, typename S, typename... Args>
void error_and_throw [[noreturn]] (const S & format, Args &&... args)
{
  std::string message;
  if constexpr(sizeof...(Args) == 0) { message = fmt::format("{}", format); }
  else
  {
    message = fmt::format(fmt::runtime(format), std::forward<Args>(args)...);
  }
  details::notify(message);
  details::cerr().critical(message);
  details::cerr().critical("=== Backtrace ===\n{}", MC_FMT_STREAMED(boost::stacktrace::stacktrace()));
  throw ExceptionT(message);
}

template<typename S, typename... Args>
void critical(const S & format, Args &&... args)
{
  if constexpr(sizeof...(Args) == 0) { details::cerr().critical(format); }
  else
  {
    details::cerr().critical(fmt::runtime(format), std::forward<Args>(args)...);
  }
}

template<typename S, typename... Args>
void error(const S & format, Args &&... args)
{
  if constexpr(sizeof...(Args) == 0) { details::cerr().error(format); }
  else
  {
    details::cerr().error(fmt::runtime(format), std::forward<Args>(args)...);
  }
}

template<typename S, typename... Args>
void warning(const S & format, Args &&... args)
{
  if constexpr(sizeof...(Args) == 0) { details::cerr().warn(format); }
  else
  {
    details::cerr().warn(fmt::runtime(format), std::forward<Args>(args)...);
  }
}

template<typename S, typename... Args>
void info(const S & format, Args &&... args)
{
  if constexpr(sizeof...(Args) == 0) { details::info().info(format); }
  else
  {
    details::info().info(fmt::runtime(format), std::forward<Args>(args)...);
  }
}

template<typename S, typename... Args>
void success(const S & format, Args &&... args)
{
  if constexpr(sizeof...(Args) == 0) { details::success().info(format); }
  else
  {
    details::success().info(fmt::runtime(format), std::forward<Args>(args)...);
  }
}

template<typename S, typename... Args>
void notify(const S & format, Args &&... args)
{
  if constexpr(sizeof...(Args) == 0) { details::notify(fmt::format("{}", format)); }
  else
  {
    details::notify(fmt::format(fmt::runtime(format), std::forward<Args>(args)...));
  }
}

} // namespace log

} // namespace mc_rtc

#ifndef WIN32

namespace mc_rtc
{

constexpr auto OUT_NONE = "\033[00m";
constexpr auto OUT_BLUE = "\033[01;34m";
constexpr auto OUT_GREEN = "\033[01;32m";
constexpr auto OUT_PURPLE = "\033[01;35m";
constexpr auto OUT_RED = "\033[01;31m";

} // namespace mc_rtc

#  define LOG_ERROR(args)                                                                \
    _Pragma("GCC warning \"This macro is deprecated, use mc_rtc::log::error instead\""); \
    std::cerr << mc_rtc::OUT_RED << args << mc_rtc::OUT_NONE << "\n";
#  define LOG_WARNING(args)                                                                \
    _Pragma("GCC warning \"This macro is deprecated, use mc_rtc::log::warning instead\""); \
    std::cerr << mc_rtc::OUT_PURPLE << args << mc_rtc::OUT_NONE << "\n";
#  define LOG_INFO(args)                                                                \
    _Pragma("GCC warning \"This macro is deprecated, use mc_rtc::log::info instead\""); \
    std::cout << mc_rtc::OUT_BLUE << args << mc_rtc::OUT_NONE << "\n";
#  define LOG_SUCCESS(args)                                                                \
    _Pragma("GCC warning \"This macro is deprecated, use mc_rtc::log::success instead\""); \
    std::cout << mc_rtc::OUT_GREEN << args << mc_rtc::OUT_NONE << "\n";

#  define LOG_ERROR_AND_THROW(exception_type, args)                                                                  \
    {                                                                                                                \
      _Pragma("GCC warning \"This macro is deprecated, use mc_rtc::log::error_and_throw<exception_type> instead\""); \
      std::stringstream strstrm;                                                                                     \
      strstrm << args;                                                                                               \
      LOG_ERROR(strstrm.str())                                                                                       \
      throw exception_type(strstrm.str());                                                                           \
    }

#else

#  include <windows.h>
namespace mc_rtc
{
static const HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
constexpr auto OUT_NONE = 15;
constexpr auto OUT_BLUE = 11;
constexpr auto OUT_GREEN = 10;
constexpr auto OUT_PURPLE = 13;
constexpr auto OUT_RED = 12;
} // namespace mc_rtc

#  define __MC_RTC_STR2__(x) #x
#  define __MC_RTC_STR1__(x) __MC_RTC_STR2__(x)
#  define __MC_RTC_PRAGMA_LOC__ __FILE__ "("__MC_RTC_STR1__(__LINE__) ") "

#  define LOG_ERROR(args)                                                                                           \
    __pragma(message(__MC_RTC_PRAGMA_LOC__ ": warning: this macro is deprecated, use mc_rtc::log::error instead")); \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_RED);                                                     \
    std::cerr << args << std::endl;                                                                                 \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_NONE);

#  define LOG_WARNING(args)                                                                                           \
    __pragma(message(__MC_RTC_PRAGMA_LOC__ ": warning: this macro is deprecated, use mc_rtc::log::warning instead")); \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_PURPLE);                                                    \
    std::cerr << args << std::endl;                                                                                   \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_NONE);

#  define LOG_INFO(args)                                                                                           \
    __pragma(message(__MC_RTC_PRAGMA_LOC__ ": warning: this macro is deprecated, use mc_rtc::log::info instead")); \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_BLUE);                                                   \
    std::cout << args << std::endl;                                                                                \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_NONE);

#  define LOG_SUCCESS(args)                                                                                           \
    __pragma(message(__MC_RTC_PRAGMA_LOC__ ": warning: this macro is deprecated, use mc_rtc::log::success instead")); \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_GREEN);                                                     \
    std::cout << args << std::endl;                                                                                   \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_NONE);

#  define LOG_ERROR_AND_THROW(exception_type, args)                                                                  \
    {                                                                                                                \
      __pragma(                                                                                                      \
          message(__MC_RTC_PRAGMA_LOC__                                                                              \
                  ": warning: this macro is deprecated, use mc_rtc::log::error_and_throw<exception_type> instead")); \
      std::stringstream strstrm;                                                                                     \
      strstrm << args;                                                                                               \
      LOG_ERROR(strstrm.str())                                                                                       \
      throw exception_type(strstrm.str());                                                                           \
    }

#endif
