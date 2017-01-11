#pragma once

#include <mc_rbdyn/api.h>

#include <string>
#include <vector>

namespace mc_rbdyn
{

struct MC_RBDYN_DLLAPI Springs
{
public:
  Springs() : springsBodies(0), afterSpringsBodies(0), springsJoints(0) {}
public:
  std::vector<std::string> springsBodies;
  std::vector<std::string> afterSpringsBodies;
  std::vector< std::vector<std::string> > springsJoints;
};

}
