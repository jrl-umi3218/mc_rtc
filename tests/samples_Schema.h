#pragma once

/** Contains some sample schemas to share among tests */

#include <mc_rtc/Schema.h>

struct SimpleSchema : public mc_rtc::schema::Schema<SimpleSchema>
{
#define MEMBER(...) SCHEMA_REQUIRED_DEFAULT_MEMBER(SimpleSchema, __VA_ARGS__)
  MEMBER(bool, useFeature, "Use magic feature");
  MEMBER(double, weight, "Task weight");
  MEMBER(std::vector<std::string>, names, "Some names");
  MEMBER(sva::ForceVecd, wrench, "Target wrench");
  MEMBER(sva::PTransformd, pt, "Some transform");
#undef MEMBER
};
