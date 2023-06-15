/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "gui_TestServer.h"

struct AdvancedFormServer : public TestServer
{
  AdvancedFormServer()
  {
    builder.addElement({}, mc_rtc::gui::Form(
                               "Send data", [this](const mc_rtc::Configuration & data) { callback(data); },
                               mc_rtc::gui::FormPoint3DInput("Point3D", true, Eigen::Vector3d::Zero()),
                               mc_rtc::gui::FormRotationInput("Rotation", true, {Eigen::Vector3d(1, 0, 0)}),
                               mc_rtc::gui::FormTransformInput("Transform", true, {Eigen::Vector3d(0, 0, 1)})));
  }

  void callback(const mc_rtc::Configuration & data)
  {
    mc_rtc::log::info("Data from callback:\n{}", data.dump(true, true));
  }
};

int main()
{
  AdvancedFormServer server;
  server.loop();
  return 0;
}
