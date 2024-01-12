#pragma once

/** Contains some sample schemas to share among tests */

#include <mc_rtc/Schema.h>

struct SimpleSchema
{
  MC_RTC_NEW_SCHEMA(SimpleSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(SimpleSchema, __VA_ARGS__))
  MEMBER(bool, useFeature, "Use magic feature")
  MEMBER(double, weight, "Task weight")
  MEMBER(std::vector<std::string>, names, "Some names")
  using MapType = std::map<std::string, double>;
  MEMBER(MapType, jointValues, "Some joint values")
  MEMBER(sva::ForceVecd, wrench, "Target wrench")
  MEMBER(sva::PTransformd, pt, "Some transform")
  MEMBER(Eigen::Vector2d, v2d, "Some 2d setting")
  MEMBER(Eigen::Vector4d, v4d, "Some 4d setting")
#undef MEMBER
};
