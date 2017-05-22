/** This file holds all mc_rtc::Configuration based serialization operation for mc_rbdyn objects */

#include <mc_rtc/Configuration.h>

#include <mc_rbdyn/Base.h>

namespace sva
{

inline void load_object(const mc_rtc::Configuration & config, sva::PTransformd & pt)
{
  pt.translation() = config("translation");
  pt.rotation() = config("rotation");
}

inline mc_rtc::Configuration save_object(const sva::PTransformd & pt)
{
  mc_rtc::Configuration config;
  config.add("translation", pt.translation());
  config.add("rotation", pt.rotation());
  return config;
}

}

namespace mc_rbdyn
{

inline void load_object(const mc_rtc::Configuration & config, Base & b)
{
  b.baseName = static_cast<std::string>(config("baseName"));
  b.X_0_s = config("X_0_s");
  b.X_b0_s = config("X_b0_s");
  b.baseType = static_cast<rbd::Joint::Type>(static_cast<int>(config("baseType")));
}

inline mc_rtc::Configuration save_object(const Base & b)
{
  mc_rtc::Configuration config;
  config.add("baseName", b.baseName);
  config.add("X_0_s", b.X_0_s);
  config.add("X_b0_s", b.X_b0_s);
  config.add("baseType", static_cast<int>(b.baseType));
  return config;
}

}
