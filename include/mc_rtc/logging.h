#pragma once

namespace mc_rtc
{

constexpr auto OUT_NONE = "\033[00m";
constexpr auto OUT_BLUE = "\033[01;34m";
constexpr auto OUT_GREEN = "\033[01;32m";
constexpr auto OUT_PURPLE = "\033[01;35m";
constexpr auto OUT_RED = "\033[01;31m";

}

#define LOG_ERROR(args)\
  std::cerr << mc_rtc::OUT_RED << args << mc_rtc::OUT_NONE << std::endl;
#define LOG_WARNING(args)\
  std::cerr << mc_rtc::OUT_PURPLE << args << mc_rtc::OUT_NONE << std::endl;
#define LOG_INFO(args)\
  std::cout << mc_rtc::OUT_BLUE << args << mc_rtc::OUT_NONE << std::endl;
#define LOG_SUCCESS(args)\
  std::cout << mc_rtc::OUT_GREEN << args << mc_rtc::OUT_NONE << std::endl;
