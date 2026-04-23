/*
 * Copyright 2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_rtc/constants.h>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <SpaceVecAlg/SpaceVecAlg>

/**
  fmt version summary:
  Ubuntu 20.04 (Focal)	6.1.2
  Ubuntu 22.04 (Jammy)	8.1.1 -> has std::ostream support
  Ubuntu 24.04 (Noble)	9.1.0 -> automatics std::ostream support was removed
*/

#if FMT_VERSION >= 9 * 10000
// fmt 9.0.0 removed automated operator<< discovery we use fmt::streamed instead
// when needed through a macro
#  define MC_FMT_STREAMED(X) fmt::streamed(X)

#  include <boost/filesystem.hpp>
#  include <filesystem>

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

// Formatter for std::filesystem::path
template<>
struct fmt::formatter<std::filesystem::path> : fmt::formatter<std::string>
{
  template<typename FormatContext>
  auto format(const std::filesystem::path & p, FormatContext & ctx)
  {
    return fmt::formatter<std::string>::format(p.string(), ctx);
  }
};

// Formatter for any Eigen type derived from EigenBase (matrices, arrays,
// expressions, etc.) Tested for fmt_9, fmt_10, fmt_11 and fmt_12
template<typename T>
struct fmt::formatter<T, char, std::enable_if_t<std::is_base_of_v<Eigen::EigenBase<T>, T>>> : fmt::ostream_formatter
{
};

// Prevent fmt/ranges from also treating Eigen types as ranges. Otherwise fmt
// finds two competing formatter specializations (ranges + Eigen ostream) and
// fails with an ambiguous formatter instantiation.
template<typename T, typename Char>
struct fmt::range_format_kind<T, Char, std::enable_if_t<std::is_base_of_v<Eigen::EigenBase<T>, T>>>
: std::integral_constant<fmt::range_format, fmt::range_format::disabled>
{
};

#  define MC_FMT_OSTREAM_FORMATTER(TYPE)                 \
    template<>                                           \
    struct fmt::formatter<TYPE> : fmt::ostream_formatter \
    {                                                    \
    };

// Automatic ostream formatter for SpaceVecAlg types
MC_FMT_OSTREAM_FORMATTER(sva::ABInertiad)
MC_FMT_OSTREAM_FORMATTER(sva::AdmittanceVecd)
MC_FMT_OSTREAM_FORMATTER(sva::ForceVecd)
MC_FMT_OSTREAM_FORMATTER(sva::MotionVecd)
MC_FMT_OSTREAM_FORMATTER(sva::RBInertiad)
MC_FMT_OSTREAM_FORMATTER(sva::PTransformd)
MC_FMT_OSTREAM_FORMATTER(sva::ImpedanceVecd)
#else // FMT_VERSION < 9 * 10000
/* std::ostream support was automatic in fmt versions < 9,
   thus fmt::streamed did not exist and was not needed (which IMHO was a much
   saner default) */
#  define MC_FMT_STREAMED(X) X
#endif
