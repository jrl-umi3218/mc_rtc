#pragma once

#include <mc_rbdyn/api.h>

#include <Eigen/Geometry>
#include <memory>
#include <vector>

namespace geos
{
namespace geom
{
class Geometry;
}
} // namespace geos

namespace mc_rbdyn
{

struct MC_RBDYN_DLLAPI QuadraticGenerator
{
  QuadraticGenerator(double start, double end, unsigned int nrSteps, unsigned int proportion = 4);

  void next(double & percentOut, double & speedOut);

private:
  double start;
  double end;
  unsigned int nrSteps_;
  unsigned int proportion;
  unsigned int current;
  double s1;
  double s2;
  unsigned int t1;
  unsigned int t2;
  double max_speed;
};

struct MC_RBDYN_DLLAPI Plane
{
  Eigen::Vector3d normal;
  double offset;
};

MC_RBDYN_DLLAPI std::vector<Plane> planes_from_polygon(const std::shared_ptr<geos::geom::Geometry> & geometry);

MC_RBDYN_DLLAPI std::vector<Eigen::Vector3d> points_from_polygon(std::shared_ptr<geos::geom::Geometry> geometry);

} // namespace mc_rbdyn
