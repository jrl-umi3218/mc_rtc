/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/GUIState.h>

#include <boost/test/unit_test.hpp>

struct DummyProvider
{
  double value = 42.0;
  Eigen::Vector3d point = Eigen::Vector3d(0., 1., 2.);
};

BOOST_AUTO_TEST_CASE(TestGUIStateBuilder)
{
  DummyProvider provider;
  mc_rtc::gui::StateBuilder builder;
  builder.addElement({"dummy", "provider"}, mc_rtc::gui::Label("value", [&provider] { return provider.value; }));
  builder.addElement({"dummy", "provider"}, mc_rtc::gui::ArrayLabel("point", [&provider] { return provider.point; }));
  {
    const auto & state = builder.update();
    BOOST_REQUIRE(state.has("STATE"));
    BOOST_REQUIRE(state("STATE").has("_sub"));
    BOOST_REQUIRE(state("STATE")("_sub").has("dummy"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy").has("_sub"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub").has("provider"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub")("provider").has("point"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub")("provider")("point").has("data"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub")("provider")("point")("data") == provider.point);
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub")("provider").has("value"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub")("provider")("value").has("data"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub")("provider")("value")("data") == provider.value);
  }
  {
    provider.value = -122.5;
    provider.point = Eigen::Vector3d::Random();
    const auto & state = builder.update();
    BOOST_REQUIRE(state.has("STATE"));
    BOOST_REQUIRE(state("STATE").has("_sub"));
    BOOST_REQUIRE(state("STATE")("_sub").has("dummy"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy").has("_sub"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub").has("provider"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub")("provider").has("point"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub")("provider")("point").has("data"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub")("provider")("point")("data") == provider.point);
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub")("provider").has("value"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub")("provider")("value").has("data"));
    BOOST_REQUIRE(state("STATE")("_sub")("dummy")("_sub")("provider")("value")("data") == provider.value);
  }
}
