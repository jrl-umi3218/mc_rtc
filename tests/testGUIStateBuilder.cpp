/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/StateBuilder.h>

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
  std::vector<char> buffer;
  auto s = builder.update(buffer);
  std::cout << "state size " << s << "\n";
  auto state = mc_rtc::Configuration::fromMessagePack(buffer.data(), s);
  std::cout << state.dump(true) << "\n";
}
