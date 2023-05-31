#include <mc_rtc/Schema.h>

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(TestDefault)
{
  using namespace mc_rtc::schema::details;
  // Arithmetic types default to zero
  static_assert(Default<int>::value == 0);
  static_assert(Default<uint64_t>::value == 0);
  static_assert(Default<float>::value == 0.0f);
  static_assert(Default<double>::value == 0.0);
  // Vectors default to Zero
  BOOST_REQUIRE(Default<Eigen::Vector3d>::value == Eigen::Vector3d::Zero());
  BOOST_REQUIRE(Default<Eigen::Vector6d>::value == Eigen::Vector6d::Zero());
  using Vector9f = Eigen::Matrix<float, 9, 1>;
  BOOST_REQUIRE(Default<Vector9f>::value == Vector9f::Zero());
  // Square matrixes default to identity
  BOOST_REQUIRE(Default<Eigen::Matrix3d>::value == Eigen::Matrix3d::Identity());
  BOOST_REQUIRE(Default<Eigen::Matrix3i>::value == Eigen::Matrix3i::Identity());
  // String defaults to empty
  BOOST_REQUIRE(Default<std::string>::value == std::string(""));
  // SpaceVecAlg objects default to reasonable values
  BOOST_REQUIRE(Default<sva::PTransformd>::value == sva::PTransformd::Identity());
  BOOST_REQUIRE(Default<sva::MotionVecd>::value == sva::MotionVecd::Zero());
  BOOST_REQUIRE(Default<sva::ForceVecd>::value == sva::ForceVecd::Zero());
  BOOST_REQUIRE(Default<sva::ImpedanceVecd>::value == sva::ImpedanceVecd::Zero());
  BOOST_REQUIRE(Default<sva::AdmittanceVecd>::value == sva::AdmittanceVecd::Zero());
}
