/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/utils_api.h>

#include <iostream>

#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/logger.h>

// fmt 9.0.0 removed automated operator<< discovery we use fmt::streamed instead when needed through a macro
#if FMT_VERSION >= 9 * 10000
#  define MC_FMT_STREAMED(X) fmt::streamed(X)
#  include <boost/filesystem.hpp>
#  include <Eigen/Core>
#  include <fmt/ostream.h>
#  include <fmt/ranges.h>
#  include <type_traits>

// Formatter for Eigen dense types (like Eigen::Matrix, Eigen::Array)
template<typename T, typename Char>
struct fmt::formatter<T,
                      Char,
                      std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>
                                       && (fmt::range_format_kind<T, Char, void>::value == fmt::range_format::disabled)>>
: fmt::ostream_formatter
{
};

// Formatter for boost::filesystem::path
template<>
struct fmt::formatter<boost::filesystem::path> : fmt::formatter<std::string>
{
  template<typename FormatContext>
  auto format(const boost::filesystem::path & p, FormatContext & ctx)
  {
    return fmt::formatter<std::string>::format(p.string(), ctx);
  }
};

#else
#  define MC_FMT_STREAMED(X) X
#endif

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

template<typename ExceptionT = std::runtime_error, typename... Args>
void error_and_throw [[noreturn]] (Args &&... args)
{
  auto message = fmt::format(std::forward<Args>(args)...);
  details::notify(message);
  details::cerr().critical(message);
  details::cerr().critical("=== Backtrace ===\n{}", MC_FMT_STREAMED(boost::stacktrace::stacktrace()));
  throw ExceptionT(message);
}

template<typename... Args>
void critical(Args &&... args)
{
  details::cerr().critical(std::forward<Args>(args)...);
}

template<typename... Args>
void error(Args &&... args)
{
  details::cerr().error(std::forward<Args>(args)...);
}

template<typename... Args>
void warning(Args &&... args)
{
  details::cerr().warn(std::forward<Args>(args)...);
}

template<typename... Args>
void info(Args &&... args)
{
  details::info().info(std::forward<Args>(args)...);
}

template<typename... Args>
void success(Args &&... args)
{
  details::success().info(std::forward<Args>(args)...);
}

template<typename... Args>
void notify(Args &&... args)
{
  details::notify(fmt::format(std::forward<Args>(args)...));
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
