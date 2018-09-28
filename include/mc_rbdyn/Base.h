#pragma once

#include <mc_rbdyn/api.h>

#include <RBDyn/Joint.h>

#include <string>

namespace mc_rbdyn
{

/** This struct represents a base for the creation of a robot */
struct MC_RBDYN_DLLAPI Base
{
  /** Name of the base */
  std::string baseName;
  /** Transformation from the world to the root joint */
  sva::PTransformd X_0_s;
  /** Transformation from the root body to the root joint */
  sva::PTransformd X_b0_s;
  /** Type of the root joint */
  rbd::Joint::Type baseType;
};

} // namespace mc_rbdyn
