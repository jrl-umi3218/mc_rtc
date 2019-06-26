/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

#include <cstdint>
#include <string>
#include <vector>

namespace mc_solver
{

struct ContactMsg
{
  uint16_t r1_index;
  uint16_t r2_index;
  std::string r1_body;
  std::string r2_body;
  std::string r1_surface;
  std::string r2_surface;
  std::vector<sva::PTransformd> r1_points;
  sva::PTransformd X_b1_b2;
  uint16_t nr_generators;
  double mu;
};

} // namespace mc_solver
