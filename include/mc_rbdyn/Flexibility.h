#pragma once

#include <mc_rbdyn/api.h>

#include <string>

namespace mc_rbdyn
{

struct MC_RBDYN_DLLAPI Flexibility
{
public:
  std::string jointName;
  double K;
  double C;
  double O;
};

}
