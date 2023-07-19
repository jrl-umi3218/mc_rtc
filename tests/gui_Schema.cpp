/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "gui_TestServer.h"

#include "samples_Schema.h"

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
#define MEMBER(...) SCHEMA_MEMBER(InteractiveSchema, __VA_ARGS__)
  MEMBER(Eigen::Vector3d,
         point,
         "3D point",
         mc_rtc::schema::Required | mc_rtc::schema::Interactive,
         Eigen::Vector3d::Zero());
  MEMBER(Eigen::Vector3d, gain, "3D gain", mc_rtc::schema::Required, Eigen::Vector3d::Zero());
  MEMBER(Eigen::Vector3d, gainOpt, "3D optional gain", mc_rtc::schema::None, Eigen::Vector3d::Zero());
#undef MEMBER
};

struct SimpleVariant : public mc_rtc::schema::Schema<SimpleVariant>
{
  using gain_t = std::variant<double, Eigen::Vector3d>;
#define MEMBER(...) SCHEMA_REQUIRED_DEFAULT_MEMBER(SimpleVariant, __VA_ARGS__)
  MEMBER(gain_t, stiffness, "Task stiffness", mc_rtc::schema::Choices({"scalar", "dimensional"}));
#undef MEMBER
};

struct SchemaServer : public TestServer
{
  SchemaServer()
  {
    simple_.addToGUI(builder, {"Simple"}, "Update simple", [this]() { simple_updated(); });
    compose_.addToGUI(builder, {"Compose"}, "Update compose", [this]() { compose_updated(); });
    interactive_.addToGUI(builder, {"Interactive"}, "Update interactive", [this]() { interactive_updated(); });
    simple_variant_.addToGUI(builder, {"Simple variant"}, "Update simple", [this]() { simple_variant_updated(); });
  }

  SimpleSchema simple_;
  ComposeSchema compose_;
  InteractiveSchema interactive_;
  SimpleVariant simple_variant_;

  void simple_updated() { mc_rtc::log::info("simple_ updated:\n{}", simple_.dump(true, true)); }
  void compose_updated() { mc_rtc::log::info("compose_ updated:\n{}", compose_.dump(true, true)); }
  void interactive_updated() { mc_rtc::log::info("interactive_ updated:\n{}", interactive_.dump(true, true)); }
  void simple_variant_updated() { mc_rtc::log::info("simple_variant_ updated:\n{}", simple_variant_.dump(true, true)); }
};

int main()
{
  SchemaServer server;
  server.loop();
  return 0;
}
