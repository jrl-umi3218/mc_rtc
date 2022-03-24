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
  std::vector<char> buffer;
  // Size of the GUI message when the GUI is empty
  auto empty_size = builder.update(buffer);
  builder.addElement({"dummy", "provider"}, mc_rtc::gui::Label("value", [&provider] { return provider.value; }));
  builder.addElement({"dummy", "provider"}, mc_rtc::gui::ArrayLabel("point", [&provider] { return provider.point; }));
  // Check that after adding elements we have a different size, ref_size is our full message
  auto ref_size = builder.update(buffer);
  {
    BOOST_REQUIRE(ref_size != empty_size);
  }
  builder.removeElement({"dummy", "provider"}, "value");
  builder.removeElement({"dummy", "provider"}, "point");
  // Removing all elements manually sends us back to the empty state
  {
    auto s = builder.update(buffer);
    BOOST_REQUIRE(s == empty_size);
  }
  // Add the elements back
  builder.addElement({"dummy", "provider"}, mc_rtc::gui::Label("value", [&provider] { return provider.value; }));
  builder.addElement({"dummy", "provider"}, mc_rtc::gui::ArrayLabel("point", [&provider] { return provider.point; }));
  {
    auto s = builder.update(buffer);
    BOOST_REQUIRE(s == ref_size);
  }
  // Removing through the category should clean up the full GUI
  builder.removeCategory({"dummy", "provider"});
  {
    auto s = builder.update(buffer);
    BOOST_REQUIRE(s == empty_size);
  }
  // Now we add them with explicit sources
  builder.addElement(&provider, {"dummy", "provider"},
                     mc_rtc::gui::Label("value", [&provider] { return provider.value; }));
  builder.addElement(&provider, {"dummy", "provider"},
                     mc_rtc::gui::ArrayLabel("point", [&provider] { return provider.point; }));
  {
    auto s = builder.update(buffer);
    BOOST_REQUIRE(s == ref_size);
  }
  // We remove by providing the source only
  builder.removeElements(&provider);
  {
    auto s = builder.update(buffer);
    BOOST_REQUIRE(s == empty_size);
  }
  builder.addElement(&provider, {"dummy"}, mc_rtc::gui::Label("value", [&provider] { return provider.value; }));
  // New ref_size for the GUI with one element
  ref_size = builder.update(buffer);
  BOOST_REQUIRE(ref_size != empty_size);
  builder.addElement(&provider, {"dummy", "provider"},
                     mc_rtc::gui::ArrayLabel("point", [&provider] { return provider.point; }));
  {
    auto s = builder.update(buffer);
    BOOST_REQUIRE(s != ref_size && s != empty_size);
  }
  // Remove elements within a category using a source
  builder.removeElements({"dummy", "provider"}, &provider);
  {
    auto s = builder.update(buffer);
    BOOST_REQUIRE(s == ref_size);
  }
  builder.removeElements({"dummy"}, &provider);
  {
    auto s = builder.update(buffer);
    BOOST_REQUIRE(s == empty_size);
  }
  // Update ref_size
  builder.addElement(&provider, {"dummy"}, mc_rtc::gui::Label("value", [&provider] { return provider.value; }));
  builder.addElement(&provider, {"dummy", "provider"},
                     mc_rtc::gui::ArrayLabel("point", [&provider] { return provider.point; }));
  ref_size = builder.update(buffer);
  {
    BOOST_REQUIRE(ref_size != empty_size);
  }
  // Remove only the elements of a given category (no recursion)
  builder.removeElements({"dummy"}, &provider, false);
  {
    auto s = builder.update(buffer);
    BOOST_REQUIRE(s != empty_size);
  }
  // Restore the message
  builder.addElement(&provider, {"dummy"}, mc_rtc::gui::Label("value", [&provider] { return provider.value; }));
  {
    auto s = builder.update(buffer);
    BOOST_REQUIRE(s == ref_size);
  }
  // Remove the elements of a given category with recursion
  builder.removeElements({"dummy"}, &provider, true);
  {
    auto s = builder.update(buffer);
    BOOST_REQUIRE(s == empty_size);
  }
}
