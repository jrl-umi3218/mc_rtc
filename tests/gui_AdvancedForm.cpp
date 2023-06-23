/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "gui_TestServer.h"

struct AdvancedFormServer : public TestServer
{
  AdvancedFormServer()
  {
    builder.addElement({"3D elements"},
                       mc_rtc::gui::Form(
                           "Send data", [this](const mc_rtc::Configuration & data) { callback(data); },
                           mc_rtc::gui::FormPoint3DInput("Point3D", true, Eigen::Vector3d::Zero()),
                           mc_rtc::gui::FormRotationInput("Rotation", true, {Eigen::Vector3d(1, 0, 0)}),
                           mc_rtc::gui::FormTransformInput("Transform", true, {Eigen::Vector3d(0, 0, 1)})));
    builder.addElement({"Object"},
                       mc_rtc::gui::Form(
                           "Send data", [this](const mc_rtc::Configuration & data) { callback(data); },
                           mc_rtc::gui::FormObjectInput("nested", true, mc_rtc::gui::FormStringInput("name", true))));
    builder.addElement({"ObjectArray"}, mc_rtc::gui::Form(
                                            "Send data", [this](const mc_rtc::Configuration & data) { callback(data); },
                                            mc_rtc::gui::FormObjectArrayInput(
                                                "array of objects", true,
                                                mc_rtc::gui::FormComboInput("name", true, {"a", "b", "c", "d"}),
                                                mc_rtc::gui::FormNumberInput("number", false, 42.42))));
    builder.addElement(
        {"GenericArray"},
        mc_rtc::gui::Form(
            "Send data", [this](const mc_rtc::Configuration & data) { callback(data); },
            mc_rtc::gui::FormGenericArrayInput("array of strings", true, mc_rtc::gui::FormStringInput("name", true))));
    builder.addElement(
        {"GenericArray with data"},
        mc_rtc::gui::Form(
            "Send data", [this](const mc_rtc::Configuration & data) { callback(data); },
            mc_rtc::gui::FormGenericArrayInput("array of strings", true, mc_rtc::gui::FormStringInput("name", true),
                                               [this]() -> const std::vector<std::string> & { return str_vector_; })));
    builder.addElement(
        {"Trajectory maker", "Point3D"},
        mc_rtc::gui::Form(
            "Send data", [this](const mc_rtc::Configuration & data) { callback(data); },
            mc_rtc::gui::FormGenericArrayInput("trajectory", true,
                                               mc_rtc::gui::FormPoint3DInput("point", true, Eigen::Vector3d::Zero()))));
    builder.addElement(
        {"Trajectory maker", "Transform"},
        mc_rtc::gui::Form(
            "Send data", [this](const mc_rtc::Configuration & data) { callback(data); },
            mc_rtc::gui::FormGenericArrayInput(
                "trajectory", true, mc_rtc::gui::FormTransformInput("point", true, sva::PTransformd::Identity()))));
    builder.addElement({"OneOf"},
                       mc_rtc::gui::Form(
                           "Send data", [this](const mc_rtc::Configuration & data) { callback(data); },
                           mc_rtc::gui::FormOneOfInput("oneof", true, mc_rtc::gui::FormCheckbox("bool", true, false),
                                                       mc_rtc::gui::FormNumberInput("number", true, 42.42),
                                                       mc_rtc::gui::FormStringInput("string", true, "empty"))));
  }

  void callback(const mc_rtc::Configuration & data)
  {
    mc_rtc::log::info("Data from callback:\n{}", data.dump(true, true));
  }

  std::vector<std::string> str_vector_ = {"a", "b", "c", "d"};
};

int main()
{
  AdvancedFormServer server;
  server.loop();
  return 0;
}
