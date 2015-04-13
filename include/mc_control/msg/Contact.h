#ifndef _H_CONTROLMSGCONTACT_H_
#define _H_CONTROLMSGCONTACT_H_

#include <cstdint>
#include <vector>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_control
{

struct ContactMsg
{
  std::string robot_surf;
  std::string env_surf;
  std::vector<sva::PTransformd> robot_surf_points;
  uint16_t nr_generators;
  double mu;
};

}

#endif
