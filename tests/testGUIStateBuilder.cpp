#include <boost/test/unit_test.hpp>

#include <mc_rtc/GUIState.h>

struct DummyProvider
{
  double value = 42.0;
  Eigen::Vector3d point = Eigen::Vector3d(0., 1., 2.);
};

BOOST_AUTO_TEST_CASE(TestGUIStateBuilder)
{
  DummyProvider provider;
  mc_rtc::gui::StateBuilder builder;
  builder.addElement(
   mc_rtc::gui::Element<Eigen::Vector3d>{
      {"dummy", "provider", "point"},
      [&provider]() { return provider.point; }
   }
  );
  builder.addElement(
   mc_rtc::gui::Element<double>{
      {"dummy", "provider", "value"},
      [&provider]() { return provider.value; }
   },
   mc_rtc::gui::Input<double>({}, 0.0, 1.0)
  );
  {
    const auto & state = builder.updateState().state;
    BOOST_REQUIRE(state.has("dummy"));
    BOOST_REQUIRE(state("dummy").has("provider"));
    BOOST_REQUIRE(state("dummy")("provider").has("point"));
    BOOST_REQUIRE(state("dummy")("provider")("point").has("data"));
    BOOST_REQUIRE(state("dummy")("provider")("point")("data") == provider.point);
    BOOST_REQUIRE(state("dummy")("provider").has("value"));
    BOOST_REQUIRE(state("dummy")("provider")("value").has("data"));
    BOOST_REQUIRE(state("dummy")("provider")("value")("data") == provider.value);
  }
  {
    provider.value = -122.5;
    provider.point = Eigen::Vector3d::Random();
    const auto & state = builder.updateState().state;
    BOOST_REQUIRE(state.has("dummy"));
    BOOST_REQUIRE(state("dummy").has("provider"));
    BOOST_REQUIRE(state("dummy")("provider").has("point"));
    BOOST_REQUIRE(state("dummy")("provider")("point").has("data"));
    BOOST_REQUIRE(state("dummy")("provider")("point")("data") == provider.point);
    BOOST_REQUIRE(state("dummy")("provider").has("value"));
    BOOST_REQUIRE(state("dummy")("provider")("value").has("data"));
    BOOST_REQUIRE(state("dummy")("provider")("value")("data") == provider.value);
  }
}
