#pragma once

#include <mc_rbdyn/api.h>

#include <string>
#include <vector>

namespace mc_rbdyn
{

/** Holds data regarding springs in a robot */
struct MC_RBDYN_DLLAPI Springs
{
  /** Bodies that have springs attached to them */
  std::vector<std::string> springsBodies = {};
  /** Bodies that come after the bodies that have springs attached to them */
  std::vector<std::string> afterSpringsBodies = {};
  /** Joints forming the springs */
  std::vector< std::vector<std::string> > springsJoints = {};
};

}
