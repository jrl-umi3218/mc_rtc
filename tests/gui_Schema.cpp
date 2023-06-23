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

struct SchemaServer : public TestServer
{
  SchemaServer()
  {
    simple_.addToGUI(builder, {"Simple"}, "Update simple", [this]() { simple_updated(); });
    compose_.addToGUI(builder, {"Compose"}, "Update compose", [this]() { compose_updated(); });
  }

  SimpleSchema simple_;
  ComposeSchema compose_;

  void simple_updated() { mc_rtc::log::info("simple_ updated:\n{}", simple_.dump(true, true)); }
  void compose_updated() { mc_rtc::log::info("compose_ updated:\n{}", compose_.dump(true, true)); }
};

int main()
{
  SchemaServer server;
  server.loop();
  return 0;
}
