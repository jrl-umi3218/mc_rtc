#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/constants.h>
#include <SpaceVecAlg/PTransform.h>
#include <fmt/format.h>

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

template<>
struct fmt::formatter<sva::PTransform<double>> : fmt::formatter<std::string>
{
  int precision = 3; /// default precision

  template<typename FormatContext>
  auto format(const sva::PTransform<double> & pt, FormatContext & ctx) const
  {
    const auto & rotation_ft = pt.rotation();
    auto rotation_std = pt.rotation().transpose();

    auto rpy_ft = mc_rbdyn::rpyFromMat(rotation_ft);
    auto rpy_ft_deg = rpy_ft * 180. / mc_rtc::constants::PI;

    auto rpy_std = mc_rbdyn::rpyFromMat(rotation_std);
    auto rpy_std_deg = rpy_std * 180. / mc_rtc::constants::PI;

    // Format the translation as an example
    return fmt::format_to(ctx.out(),
                          R"(PTransform:
  translation: [{0}]
  rotation:
    - matrix (Featherstone):
        {1}
    - matrix (standard):
        {2}
    - rpy (Featherstone): [{3:.{9}}] (rad), [{4:.{9}}] (deg)
    - rpy (standard):     [{5:.{9}}] (rad), [{6:.{9}}] (deg)
    )",
                          pt.translation().transpose(), // {0}
                          rotation_ft, // {1}
                          rotation_std, // {2}
                          rpy_ft, // {3}
                          rpy_ft_deg, // {4}
                          rpy_std, // {5}
                          rpy_std_deg, // {6}
                          precision // {9}
    );
  }
};
