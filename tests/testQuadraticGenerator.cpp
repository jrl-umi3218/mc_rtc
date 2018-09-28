#include <mc_rbdyn/polygon_utils.h>

#include <iostream>
#include <vector>

int main()
{
  int nr_points = 1001;
  int extra_points = 200;
  mc_rbdyn::QuadraticGenerator generator(1., 5., nr_points);
  std::vector<std::pair<double, double>> vec;
  vec.reserve(nr_points + extra_points);
  double speed, pos;
  for(int i = 0; i < nr_points + extra_points; ++i)
  {
    generator.next(pos, speed);
    vec.push_back(std::make_pair(speed, pos));
  }

  for(auto it = vec.begin(); it != vec.end(); ++it)
  {
    std::cout << it->first << " " << it->second << std::endl;
  }
}
