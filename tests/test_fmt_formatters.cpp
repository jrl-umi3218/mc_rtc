#define BOOST_TEST_MODULE FmtTortureTest
#include <mc_rtc/fmt_formatters.h>

#include <boost/test/unit_test.hpp>
#include <vector>

// Helper for generic test
template<typename T>
void check_fmt_nonempty(const std::string & label, const T & value)
{
  BOOST_CHECK(!fmt::format("{}: {}", label, value).empty());
  fmt::print("{}: {}\n", label, value);
}

#define SVA_FMT_TEST(TYPE, METHOD) check_fmt_nonempty(#TYPE, TYPE::METHOD())
#define SVA_FMT_TEST_CTOR(TYPE) check_fmt_nonempty(#TYPE, TYPE{})

BOOST_AUTO_TEST_CASE(Eigen_fixed_size_types_can_be_formatted_with_fmt)
{
  Eigen::Vector3d v(1.0, 2.0, 3.0);
  Eigen::Matrix3d m = Eigen::Matrix3d::Identity();

  BOOST_CHECK(!fmt::format("Vector: {}", v).empty());
  BOOST_CHECK(!fmt::format("Matrix:\n{}", m).empty());
  BOOST_CHECK(!fmt::format("Matrix transpose:\n{}", m.transpose()).empty());

  fmt::print("Vector: {}\n", v);
  fmt::print("Matrix: {}\n", m);
}

BOOST_AUTO_TEST_CASE(Eigen_dynamic_size_types_can_be_formatted_with_fmt)
{
  Eigen::VectorXd v(3);
  v << 1.0, 2.0, 3.0;
  Eigen::MatrixXd m(3, 3);
  m = Eigen::Matrix3d::Identity();

  BOOST_CHECK(!fmt::format("Vector: {}", v).empty());
  BOOST_CHECK(!fmt::format("Matrix:\n{}", m).empty());
  BOOST_CHECK(!fmt::format("Matrix transpose:\n{}", m.transpose()).empty());

  fmt::print("Vector: {}\n", v);
  fmt::print("Matrix: {}\n", m);
}

BOOST_AUTO_TEST_CASE(sva_PTransformd_can_be_formatted_with_fmt)
{
  Eigen::Vector3d translation(1.0, 2.0, 3.0);
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  sva::PTransformd pt(rotation, translation);

  std::string pt_str = fmt::format("PTransform:\n{}", pt);
  fmt::print("PTransform:\n{}\n", pt);

  BOOST_CHECK(!pt_str.empty());
}

BOOST_AUTO_TEST_CASE(All_sva_types_can_be_formatted_with_fmt)
{
  SVA_FMT_TEST_CTOR(sva::ABInertiad);
  SVA_FMT_TEST(sva::AdmittanceVecd, Zero);
  SVA_FMT_TEST(sva::ForceVecd, Zero);
  SVA_FMT_TEST(sva::MotionVecd, Zero);
  SVA_FMT_TEST_CTOR(sva::RBInertiad);
  SVA_FMT_TEST(sva::PTransformd, Identity);
  SVA_FMT_TEST(sva::ImpedanceVecd, Zero);
}

BOOST_AUTO_TEST_CASE(sva_function_results_can_be_formatted_with_fmt)
{
  check_fmt_nonempty("sva::RotX", sva::RotX(mc_rtc::constants::PI / 2));
  check_fmt_nonempty("sva::RotY", sva::RotY(mc_rtc::constants::PI / 2));
  check_fmt_nonempty("sva::RotZ", sva::RotZ(mc_rtc::constants::PI / 2));
}

BOOST_AUTO_TEST_CASE(fmt_ranges_is_still_active_for_standard_ranges)
{
  std::vector<int> values{1, 2, 3};
  BOOST_CHECK_EQUAL(fmt::format("{}", values), "[1, 2, 3]");
}

/**
 * Since fmt_10, format function should respect const correctness
 */
BOOST_AUTO_TEST_CASE(fmt_format_const_required_fmt_10)
{
  std::filesystem::path p = "/tmp/config";

  // This line should FAIL TO COMPILE with fmt v12 due to const-correctness
  // requirements on formatter::format.
  auto s = fmt::format("path={}", p);

  // Unreachable if compilation fails, but keeps the test "well-formed".
  BOOST_TEST(!s.empty());
}
