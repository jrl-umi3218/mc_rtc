#pragma once

#include <Eigen/Geometry>
#include <geos/geom/Polygon.h>
#include <Tasks/QPConstr.h>

namespace mc_rbdyn
{

struct QuadraticGenerator
{
  QuadraticGenerator(double start, double end, unsigned int nrSteps);

  void next(double & percentOut, double & speedOut);
private:
  double current;
  double end;
  unsigned int nrSteps;
  double speed;
  double max_speed;
  double sign;
};

struct Plane
{
  Eigen::Vector3d normal;
  double offset;
};

std::vector<Plane> planes_from_polygon(const std::shared_ptr<geos::geom::Geometry> & geometry);

void set_planes(const std::vector<Plane> & planes,
                const std::shared_ptr<tasks::qp::CoMIncPlaneConstr> & constr,
                const std::vector<Eigen::Vector3d> & speeds = {},
                const std::vector<Eigen::Vector3d> & normalsDots = {});

}
