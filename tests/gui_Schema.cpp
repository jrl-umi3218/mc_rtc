/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "gui_TestServer.h"

#include <mc_rtc/Schema.h>

struct SimpleSchema : public mc_rtc::schema::Schema<SimpleSchema>
{
#define MEMBER(...) SCHEMA_REQUIRED_DEFAULT_MEMBER(SimpleSchema, __VA_ARGS__)
  MEMBER(bool, useFeature, "Use magic feature");
  MEMBER(double, weight, "Task weight");
  MEMBER(std::vector<std::string>, names, "Some names");
#undef MEMBER
};

struct ComposeSchema : public mc_rtc::schema::Schema<ComposeSchema>
{
#define MEMBER(...) SCHEMA_REQUIRED_DEFAULT_MEMBER(ComposeSchema, __VA_ARGS__)
  MEMBER(int, integer, "Integer value");
  MEMBER(SimpleSchema, simple, "Simple schema");
  MEMBER(std::vector<SimpleSchema>, many, "Multiple simple schema");
#undef MEMBER
};

struct InteractiveSchema : public mc_rtc::schema::Schema<InteractiveSchema>
{
  using ValueFlag = mc_rtc::schema::ValueFlag;
#define MEMBER(...) SCHEMA_MEMBER(InteractiveSchema, __VA_ARGS__)
  MEMBER(Eigen::Vector3d, point, "3D point", ValueFlag::Required | ValueFlag::Interactive, Eigen::Vector3d::Zero());
  MEMBER(Eigen::Vector3d, gain, "3D gain", ValueFlag::Required, Eigen::Vector3d::Zero());
  MEMBER(Eigen::Vector3d, gainOpt, "3D optional gain", ValueFlag::None, Eigen::Vector3d::Zero());
#undef MEMBER
};

struct SchemaServer : public TestServer
{
  SchemaServer()
  {
    simple_.addToGUI(builder, {"Simple"}, "Update simple", [this]() { simple_updated(); });
    compose_.addToGUI(builder, {"Compose"}, "Update compose", [this]() { compose_updated(); });
    interactive_.addToGUI(builder, {"Interactive"}, "Update interactive", [this]() { interactive_updated(); });
  }

  SimpleSchema simple_;
  ComposeSchema compose_;
  InteractiveSchema interactive_;

  void simple_updated() { mc_rtc::log::info("simple_ updated:\n{}", simple_.dump(true, true)); }
  void compose_updated() { mc_rtc::log::info("compose_ updated:\n{}", compose_.dump(true, true)); }
  void interactive_updated() { mc_rtc::log::info("interactive_ updated:\n{}", interactive_.dump(true, true)); }
};

int main()
{
  SchemaServer server;
  server.loop();
  return 0;
}
