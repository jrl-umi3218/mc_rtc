#include <mc_rtc/constants.h>
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(TestConstants)
{
  namespace cst = mc_rtc::constants;
  double deg = 180;
  double rad = cst::PI;
  BOOST_REQUIRE(cst::toRad(deg) == rad);
  BOOST_REQUIRE(cst::toDeg(rad) == deg);

  BOOST_REQUIRE(cst::GRAVITY > 0);
}
