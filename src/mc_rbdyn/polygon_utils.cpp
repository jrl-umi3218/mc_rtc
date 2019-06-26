/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/polygon_utils.h>
#include <mc_rtc/logging.h>

#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/Polygon.h>

#include <algorithm>
#include <cmath>

namespace mc_rbdyn
{

QuadraticGenerator::QuadraticGenerator(double start, double end, unsigned int nrSteps, unsigned int proportion)
: start(start), end(end), nrSteps_(nrSteps), proportion(proportion), current(0), s1(0), s2(0)
{
  if(nrSteps_ % proportion != 0)
  {
    LOG_WARNING("nrSteps is not divisible by proportion, rounding it up to the nearest multiple");
    LOG_WARNING(nrSteps);
    nrSteps_ += proportion - (nrSteps_ % proportion);
  }
  LOG_WARNING(nrSteps_);
  t1 = nrSteps_ / proportion;
  t2 = nrSteps_ / proportion * (proportion - 1);
  max_speed = (double)proportion / (proportion - 1);
  s1 = pow(t1, 2) / 2 * max_speed * proportion / nrSteps_;
  s2 = s1 + (t2 - t1) * max_speed;
}

void QuadraticGenerator::next(double & percentOut, double & speedOut)
{
  double speed, sample;
  if(current <= t1)
  {
    speed = current * max_speed * proportion / nrSteps_;
    sample = pow(current, 2) / 2 * (max_speed * proportion / nrSteps_);
  }
  else if(current > t1 && current <= t2)
  {
    speed = max_speed;
    sample = s1 + (current - t1) * max_speed;
  }
  else if(current > t2 && current <= nrSteps_)
  {
    speed = max_speed * (1 - (current - t2) * proportion / (double)nrSteps_);
    sample = s2 + (current - t2) * max_speed - pow(current - t2, 2) / 2 * max_speed * proportion / nrSteps_;
  }
  else
  {
    // current > nrSteps_
    speed = 0.;
    sample = nrSteps_;
  }
  current += 1;
  percentOut = start + (end - start) * sample / nrSteps_;
  speedOut = (end - start) * speed / nrSteps_;
}

std::vector<Plane> planes_from_polygon(const std::shared_ptr<geos::geom::Geometry> & geometry)
{
  std::vector<Plane> res;
  geos::geom::Polygon * polygon = dynamic_cast<geos::geom::Polygon *>(geometry.get());
  if(polygon == nullptr)
  {
    LOG_ERROR("Could not cast geos::geom::Geometry to geos::geom::Polygon");
    return res;
  }
  const geos::geom::CoordinateSequence * seq = polygon->getExteriorRing()->getCoordinates();
  for(size_t i = 0; i < seq->size() - 1; ++i)
  {
    Plane plane;
    const geos::geom::Coordinate & p = seq->getAt(i);
    const geos::geom::Coordinate & prev = seq->getAt(i == 0 ? seq->size() - 1 : i - 1);
    plane.normal = Eigen::Vector3d(p.y - prev.y, prev.x - p.x, 0);
    double norm = plane.normal.norm();
    if(norm > 0)
    {
      plane.normal = plane.normal / norm;
    }
    else
    {
      plane.normal = Eigen::Vector3d::Zero();
    }
    plane.offset = -1 * (plane.normal.x() * p.x + plane.normal.y() * p.y);
    res.push_back(plane);
  }
  return res;
}

std::vector<Eigen::Vector3d> points_from_polygon(std::shared_ptr<geos::geom::Geometry> geometry)
{
  std::vector<Eigen::Vector3d> poly;
  geos::geom::Polygon * polyIn = dynamic_cast<geos::geom::Polygon *>(geometry.get());
  if(polyIn)
  {
    const geos::geom::CoordinateSequence * seq = polyIn->getExteriorRing()->getCoordinates();
    for(size_t i = 0; i < seq->size(); ++i)
    {
      const geos::geom::Coordinate & p = seq->getAt(i);
      poly.emplace_back(p.x, p.y, 0);
    }
  }
  return poly;
}

} // namespace mc_rbdyn
