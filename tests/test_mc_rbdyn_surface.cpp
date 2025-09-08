#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/Surface.h>
#include <boost/test/unit_test.hpp>
#include <tinyxml2.h>

/// This test checks all main fields, geometry, and round-trip serialization for all surface types.
/// - Checks all main fields (name, bodyName,
///  materialName, and geometry-specific fields).
/// - Checks points and attributes for equality.
/// - Verifies round-trip: XML → object → XML.
/// - Uses `tinyxml2` to parse the output XML and re-checks the fields.
BOOST_AUTO_TEST_CASE(TestSurfaceFromXMLAndToXML_Thorough)
{
  // PlanarSurface
  const char * planar_xml = "<planar_surface name=\"Top\" link=\"box_table\">"
                            "<origin rpy=\"0.1 0.2 0.3\" xyz=\"1.0 2.0 3.0\" />"
                            "<points>"
                            "<point xy=\"-0.1 -0.2\" />"
                            "<point xy=\"0.1 -0.2\" />"
                            "<point xy=\"0.1 0.2\" />"
                            "<point xy=\"-0.1 0.2\" />"
                            "</points>"
                            "<material name=\"plastic\" />"
                            "</planar_surface>";

  tinyxml2::XMLDocument doc1;
  doc1.Parse(planar_xml);
  auto * elem1 = doc1.FirstChildElement("planar_surface");
  auto surf1 = mc_rbdyn::Surface::fromXML(*elem1);
  BOOST_REQUIRE(surf1);
  BOOST_CHECK_EQUAL(surf1->name(), "Top");
  BOOST_CHECK_EQUAL(surf1->bodyName(), "box_table");
  BOOST_CHECK_EQUAL(surf1->materialName(), "plastic");
  auto * ps = dynamic_cast<mc_rbdyn::PlanarSurface *>(surf1.get());
  BOOST_REQUIRE(ps);
  BOOST_CHECK_CLOSE(ps->X_b_s().translation().x(), 1.0, 1e-8);
  BOOST_CHECK_CLOSE(ps->X_b_s().translation().y(), 2.0, 1e-8);
  BOOST_CHECK_CLOSE(ps->X_b_s().translation().z(), 3.0, 1e-8);
  BOOST_CHECK_EQUAL(ps->planarPoints().size(), 4);
  BOOST_CHECK_CLOSE(ps->planarPoints()[0].first, -0.1, 1e-8);
  BOOST_CHECK_CLOSE(ps->planarPoints()[0].second, -0.2, 1e-8);

  // Round-trip: toXML and re-parse
  tinyxml2::XMLDocument doc1_out;
  auto * elem1_out = surf1->toXML(doc1_out);
  BOOST_REQUIRE(elem1_out);
  BOOST_CHECK_EQUAL(std::string(elem1_out->Attribute("name")), "Top");
  BOOST_CHECK_EQUAL(std::string(elem1_out->Attribute("link")), "box_table");
  auto * origin1 = elem1_out->FirstChildElement("origin");
  BOOST_REQUIRE(origin1);
  BOOST_CHECK(origin1->Attribute("xyz"));
  BOOST_CHECK(origin1->Attribute("rpy"));
  auto * points1 = elem1_out->FirstChildElement("points");
  BOOST_REQUIRE(points1);
  int point_count = 0;
  for(auto * pt = points1->FirstChildElement("point"); pt; pt = pt->NextSiblingElement("point")) ++point_count;
  BOOST_CHECK_EQUAL(point_count, 4);

  // CylindricalSurface
  const char * cyl_xml = "<cylindrical_surface name=\"Cylinder\" link=\"cyl_link\" radius=\"0.05\" width=\"0.2\">"
                         "<origin rpy=\"0.0 0.0 0.0\" xyz=\"0.0 0.0 0.0\" />"
                         "<material name=\"metal\" />"
                         "</cylindrical_surface>";

  tinyxml2::XMLDocument doc2;
  doc2.Parse(cyl_xml);
  auto * elem2 = doc2.FirstChildElement("cylindrical_surface");
  auto surf2 = mc_rbdyn::Surface::fromXML(*elem2);
  BOOST_REQUIRE(surf2);
  BOOST_CHECK_EQUAL(surf2->name(), "Cylinder");
  BOOST_CHECK_EQUAL(surf2->bodyName(), "cyl_link");
  BOOST_CHECK_EQUAL(surf2->materialName(), "metal");
  auto * cs = dynamic_cast<mc_rbdyn::CylindricalSurface *>(surf2.get());
  BOOST_REQUIRE(cs);
  BOOST_CHECK_CLOSE(cs->radius(), 0.05, 1e-8);
  BOOST_CHECK_CLOSE(cs->width(), 0.2, 1e-8);

  tinyxml2::XMLDocument doc2_out;
  auto * elem2_out = surf2->toXML(doc2_out);
  BOOST_REQUIRE(elem2_out);
  BOOST_CHECK_EQUAL(std::string(elem2_out->Attribute("name")), "Cylinder");
  BOOST_CHECK_EQUAL(std::string(elem2_out->Attribute("link")), "cyl_link");
  BOOST_CHECK_CLOSE(std::stod(elem2_out->Attribute("radius")), 0.05, 1e-8);
  BOOST_CHECK_CLOSE(std::stod(elem2_out->Attribute("width")), 0.2, 1e-8);

  // GripperSurface
  const char * grip_xml = "<gripper_surface name=\"LeftGripper\" link=\"l_wrist\">"
                          "<origin rpy=\"3.14 0.0 0.0\" xyz=\"0.0 -0.0085 -0.095\" />"
                          "<motor rpy=\"0.0 0.0 0.0\" xyz=\"0.0 0.0 0.0\" max_torque=\"1000\" />"
                          "<points>"
                          "<origin rpy=\"-1.57 0.0 0.0\" xyz=\"0.0 0.02 0.0\" />"
                          "<origin rpy=\"1.57 0.0 0.0\" xyz=\"0.0 -0.02 0.0\" />"
                          "</points>"
                          "<material name=\"plastic\" />"
                          "</gripper_surface>";

  tinyxml2::XMLDocument doc3;
  doc3.Parse(grip_xml);
  auto * elem3 = doc3.FirstChildElement("gripper_surface");
  auto surf3 = mc_rbdyn::Surface::fromXML(*elem3);
  BOOST_REQUIRE(surf3);
  BOOST_CHECK_EQUAL(surf3->name(), "LeftGripper");
  BOOST_CHECK_EQUAL(surf3->bodyName(), "l_wrist");
  BOOST_CHECK_EQUAL(surf3->materialName(), "plastic");
  auto * gs = dynamic_cast<mc_rbdyn::GripperSurface *>(surf3.get());
  BOOST_REQUIRE(gs);
  BOOST_CHECK_CLOSE(gs->motorMaxTorque(), 1000.0, 1e-8);
  BOOST_CHECK_EQUAL(gs->pointsFromOrigin().size(), 2);

  tinyxml2::XMLDocument doc3_out;
  auto * elem3_out = surf3->toXML(doc3_out);
  BOOST_REQUIRE(elem3_out);
  BOOST_CHECK_EQUAL(std::string(elem3_out->Attribute("name")), "LeftGripper");
  BOOST_CHECK_EQUAL(std::string(elem3_out->Attribute("link")), "l_wrist");
  auto * motor3 = elem3_out->FirstChildElement("motor");
  BOOST_REQUIRE(motor3);
  BOOST_CHECK_CLOSE(std::stod(motor3->Attribute("max_torque")), 1000.0, 1e-8);
  auto * points3 = elem3_out->FirstChildElement("points");
  BOOST_REQUIRE(points3);
  int grip_point_count = 0;
  for(auto * pt = points3->FirstChildElement("origin"); pt; pt = pt->NextSiblingElement("origin")) ++grip_point_count;
  BOOST_CHECK_EQUAL(grip_point_count, 2);
}
