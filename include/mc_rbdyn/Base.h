#pragma once

#include <mc_rbdyn/api.h>

#include <RBDyn/Joint.h>

#include <string>

namespace mc_rbdyn
{

struct MC_RBDYN_DLLAPI Base
{
  std::string baseName;
  sva::PTransformd X_0_s;
  sva::PTransformd X_b0_s;
  rbd::Joint::Type baseType;
};

}
