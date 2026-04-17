/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/polygon_utils.h>

#include <iostream>
#include <vector>

int main()
{
  unsigned int nr_points = 1001;
  unsigned int extra_points = 200;
  mc_rbdyn::QuadraticGenerator generator(1., 5., nr_points);
  std::vector<std::pair<double, double>> vec;
  vec.reserve(nr_points + extra_points);
  double speed, pos;
  for(size_t i = 0; i < nr_points + extra_points; ++i)
  {
    generator.next(pos, speed);
    vec.push_back(std::make_pair(speed, pos));
  }

  for(auto it = vec.begin(); it != vec.end(); ++it)
  {
    std::cout << it->first << " " << it->second << std::endl;
  }
}
