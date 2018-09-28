#pragma once

#include <iostream>

#ifndef WIN32

namespace mc_rtc
{

constexpr auto OUT_NONE = "\033[00m";
constexpr auto OUT_BLUE = "\033[01;34m";
constexpr auto OUT_GREEN = "\033[01;32m";
constexpr auto OUT_PURPLE = "\033[01;35m";
constexpr auto OUT_RED = "\033[01;31m";

} // namespace mc_rtc

#  define LOG_ERROR(args) std::cerr << mc_rtc::OUT_RED << args << mc_rtc::OUT_NONE << std::endl;
#  define LOG_WARNING(args) std::cerr << mc_rtc::OUT_PURPLE << args << mc_rtc::OUT_NONE << std::endl;
#  define LOG_INFO(args) std::cout << mc_rtc::OUT_BLUE << args << mc_rtc::OUT_NONE << std::endl;
#  define LOG_SUCCESS(args) std::cout << mc_rtc::OUT_GREEN << args << mc_rtc::OUT_NONE << std::endl;

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

#  define LOG_ERROR(args)                                       \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_RED); \
    std::cerr << args << std::endl;                             \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_NONE);

#  define LOG_WARNING(args)                                        \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_PURPLE); \
    std::cerr << args << std::endl;                                \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_NONE);

#  define LOG_INFO(args)                                         \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_BLUE); \
    std::cout << args << std::endl;                              \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_NONE);

#  define LOG_SUCCESS(args)                                       \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_GREEN); \
    std::cout << args << std::endl;                               \
    SetConsoleTextAttribute(mc_rtc::hConsole, mc_rtc::OUT_NONE);

#endif

#define LOG_ERROR_AND_THROW(exception_type, args) \
  {                                               \
    std::stringstream strstrm;                    \
    strstrm << args;                              \
    LOG_ERROR(strstrm.str())                      \
    throw exception_type(strstrm.str());          \
  }
