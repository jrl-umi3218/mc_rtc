#pragma once

#if FMT_VERSION >= 9 * 10000

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

#endif
