#define BOOST_TEST_MODULE FmtFormattersTest
#include <mc_rtc/fmt_formatters.h>
#include <SpaceVecAlg/PTransform.h>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>
#include <fmt/format.h>

BOOST_AUTO_TEST_CASE(EigenTypesFormatting)
{
  Eigen::Vector3d v(1.0, 2.0, 3.0);
  Eigen::Matrix3d m = Eigen::Matrix3d::Identity();

  // Just check that formatting compiles
  std::string v_str = fmt::format("Vector: {}", v);
  std::string m_str = fmt::format("Matrix:\n{}", m);

  BOOST_CHECK(!v_str.empty());
  BOOST_CHECK(!m_str.empty());

  // Test mc_log_ui
  mc_rtc::log::info("Vector: {}", v);
  mc_rtc::log::info("Matrix: {}", m);
}

BOOST_AUTO_TEST_CASE(PTransformFormatting)
{
  Eigen::Vector3d translation(1.0, 2.0, 3.0);
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  sva::PTransformd pt(rotation, translation);

  std::string pt_str = fmt::format("PTransform:\n{}", pt);
  mc_rtc::log::info("PTransform:\n{}", pt);

  BOOST_CHECK(!pt_str.empty());
}
