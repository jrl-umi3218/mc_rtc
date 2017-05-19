/** This file holds all mc_rtc::Configuration based serialization operation for mc_rbdyn objects */

#include <mc_rtc/Configuration.h>

#include <mc_rbdyn/Base.h>

inline sva::PTransformd & operator<<(sva::PTransformd & pt, const mc_rtc::Configuration & config)
{
  pt.translation() = config("translation");
  pt.rotation() = config("rotation");
  return pt;
}

inline mc_rtc::Configuration & operator<<(mc_rtc::Configuration & config, const sva::PTransformd & pt)
{
  config.add("translation", pt.translation());
  config.add("rotation", pt.rotation());
  return config;
}

namespace mc_rbdyn
{

inline Base & operator<<(Base & b, const mc_rtc::Configuration & config)
{
  b.baseName = static_cast<std::string>(config("baseName"));
  b.X_0_s = config("X_0_s");
  b.X_b0_s = config("X_b0_s");
  b.baseType = static_cast<rbd::Joint::Type>(static_cast<int>(config("baseType")));
  return b;
}

inline mc_rtc::Configuration & operator<<(mc_rtc::Configuration & config, const Base & b)
{
  config.add("baseName", b.baseName);
  config.add("X_0_s", b.X_0_s);
  config.add("X_b0_s", b.X_b0_s);
  config.add("baseType", static_cast<int>(b.baseType));
  return config;
}

}
