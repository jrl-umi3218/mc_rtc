#include <boost/test/unit_test.hpp>
#include <mc_rtc_ros_compat/path.h>

BOOST_AUTO_TEST_CASE(TestRosBridge)
{
#ifdef MC_RTC_HAS_ROS_SUPPORT
  BOOST_REQUIRE_THROW(mc_rtc_ros_compat::resolve_package_path("package://does-not-exit"), std::runtime_error);
#else
  BOOST_REQUIRE_THROW(mc_rtc_ros_compat::resolve_package_path("package://does-not-exit"), std::runtime_error);
  BOOST_REQUIRE(mc_rtc_ros_compat::resolve_package_path("file:///does-not-exit") == "/does-not-exist");
  BOOST_REQUIRE(mc_rtc_ros_compat::resolve_package_path("/does-not-exit") == "/does-not-exist");
#endif
}
