/*
 * Copyright 2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotModule_visual.h>
#include <boost/test/unit_test.hpp>
#include <filesystem>
namespace fs = std::filesystem;

static auto cylinderYaml = R"(
name: cylinder
origin:
  translation: [0, 0, 0]
  rotation: [0, 0, 0]
material:
  color:
    r: 1
    g: 0
    b: 0
    a: 1
geometry:
  cylinder:
    radius: 0.5
    length: 1.0
    )";
static auto sphereYaml = R"(
name: sphere
origin:
  translation: [0, 0, 0]
  rotation: [0, 0, 0]
material:
  color:
    r: 1
    g: 0
    b: 0
    a: 1
geometry:
  sphere:
    radius: 0.5
    )";
static auto boxYaml = R"(
name: box
origin:
  translation: [0, 0, 0]
  rotation: [0, 0, 0]
material:
  color:
    r: 1
    g: 0
    b: 0
    a: 1
geometry:
  box:
    size: [1., 0.5, 2.]
# inertia is provided in the same format as RBInertiad config
# only mass should be required for basic shapes
inertia:
  mass: 10.0
    )";

BOOST_AUTO_TEST_CASE(TestRobotModuleFromVisualBox)
{
  auto boxConfig = mc_rtc::Configuration::fromYAMLData(boxYaml);
  {
    auto boxRm = mc_rbdyn::robotModuleFromVisual("box", boxConfig);
    BOOST_REQUIRE(boxRm->name == "box");
    BOOST_REQUIRE(boxRm->_visual.size() == 1);
    BOOST_REQUIRE(boxRm->_visual.count("box") == 1);
    BOOST_REQUIRE(boxRm->mb.bodies().size() == 1);
    auto & boxBody = boxRm->mb.body(0);
    BOOST_REQUIRE(boxBody.name() == "box");
    BOOST_REQUIRE(boxBody.inertia().mass() == 10.0);
    BOOST_REQUIRE(boxBody.inertia().momentum().isApprox(Eigen::Vector3d::Zero()));
    BOOST_REQUIRE(boxBody.inertia().inertia().isDiagonal());
    auto urdf_path = boxRm->urdf_path;
    BOOST_REQUIRE(!urdf_path.empty());
    BOOST_REQUIRE(fs::exists(urdf_path));
    auto rsdf_path = boxRm->rsdf_dir;
    BOOST_REQUIRE(!rsdf_path.empty());
    BOOST_REQUIRE(fs::exists(rsdf_path));
  }

  {
    // invalid inertia config on purpose
    boxConfig("inertia").add("momentum", std::vector<double>{0.1, 0.2, 0.3});
    BOOST_REQUIRE_THROW(mc_rbdyn::robotModuleFromVisual("box", boxConfig), std::runtime_error);
  }
}
