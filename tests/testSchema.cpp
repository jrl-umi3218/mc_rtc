#include "samples_Schema.h"
#include "utils.h"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(TestDefault)
{
  using namespace mc_rtc;
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
  // Variants' default is the default of the first type
  static_assert(Default<std::variant<double, Eigen::Vector3d>>::value == 0.0);
  using test_variant_t = std::variant<Eigen::Vector3d, double>;
  BOOST_REQUIRE(Default<test_variant_t>::value == Eigen::Vector3d::Zero());
  BOOST_REQUIRE(Default<std::vector<double>>::value == std::vector<double>{});
  auto map_value = Default<std::map<std::string, double>>::value;
  auto expected_map_value = std::map<std::string, double>{};
  BOOST_REQUIRE(map_value == expected_map_value);
}

BOOST_AUTO_TEST_CASE(TestSimpleSchema)
{
  SimpleSchema default_;
  SimpleSchema other_;
  BOOST_REQUIRE(default_ == other_);
  {
    default_.useFeature = true;
    default_.weight = 42.42;
    default_.names = {"a", "b", "c"};
    default_.jointValues = {{"a", 0.0}, {"b", 1.0}, {"c", 2.0}};
    default_.wrench = random_fv();
    default_.pt = random_pt();
    mc_rtc::Configuration cfg;
    default_.save(cfg);
    other_.load(cfg);
    BOOST_REQUIRE(default_ == other_);
  }
  {
    SimpleSchema new_{true, 42.42, default_.names, default_.jointValues, default_.wrench, default_.pt};
    BOOST_REQUIRE(new_ == default_);
  }
  {
    SimpleSchema copy_{default_};
    BOOST_REQUIRE(copy_ == default_);
  }
}
