#include <mc_rtc/constants.h>
#include <mc_rtc/path.h>
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

BOOST_AUTO_TEST_CASE(TestPaths_unique_path)
{
  auto pattern = std::string{"test-%%%%-%%%%"};
  auto genPath = [pattern]() { return mc_rtc::unique_path(pattern); };
  auto path1 = genPath();
  auto path2 = genPath();
  BOOST_REQUIRE_EQUAL(path1.size(), pattern.size());
  BOOST_REQUIRE_EQUAL(path1.size(), path2.size());
  BOOST_REQUIRE(std::find(path1.cbegin(), path1.cend(), '%') == path1.end());
  BOOST_REQUIRE(std::find(path2.cbegin(), path2.cend(), '%') == path2.end());
}

BOOST_AUTO_TEST_CASE(TestPaths_temp_directory_path)
{
  BOOST_REQUIRE(mc_rtc::temp_directory_path().size() != 0);
  BOOST_REQUIRE(mc_rtc::temp_directory_path("test").size() > 4);
}

BOOST_AUTO_TEST_CASE(TestPaths_user_config_directory_path)
{
  BOOST_REQUIRE(mc_rtc::user_config_directory_path().size() > 0);
  auto suffix_path = mc_rtc::user_config_directory_path("test");
  BOOST_REQUIRE(suffix_path.size() > 0);
#if __cplusplus >= 202002L
  BOOST_REQUIRE(suffix_path.ends_with("test"));
#endif
}

BOOST_AUTO_TEST_CASE(TestPaths_convertURI)
{
  auto path = std::string{"/tmp/test"};
  BOOST_REQUIRE_EQUAL(mc_rtc::convertURI(path), path);
  auto path_file = std::string{"file:///tmp/test"};
  BOOST_REQUIRE_EQUAL(mc_rtc::convertURI(path_file), path);
}
