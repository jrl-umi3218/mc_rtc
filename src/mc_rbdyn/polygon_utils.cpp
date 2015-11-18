#include <mc_rbdyn/polygon_utils.h>

#include <mc_rtc/logging.h>

#include <geos/geom/LinearRing.h>
#include <geos/geom/CoordinateSequence.h>

#include <algorithm>
#include <cmath>

namespace mc_rbdyn
{

QuadraticGenerator::QuadraticGenerator(double start, double end, unsigned int nrSteps)
: current(start), end(end), nrSteps(nrSteps), speed(0),
  max_speed(2*(end-start)/nrSteps), sign(1.)
{
}

void QuadraticGenerator::next(double & percentOut, double & speedOut)
{
  if(std::abs(current - end) > 1e-2)
  {
    current += speed;
    speed = std::max(std::min(speed + sign*2*max_speed/nrSteps, max_speed), 0.0);
    if(speed >= max_speed)
    {
      sign = -1;
      speed += sign*2*max_speed/nrSteps;
    }
    percentOut = current;
    speedOut = speed;
  }
  else
  {
    percentOut = end;
    speedOut = 0.;
  }
}

std::vector<Plane> planes_from_polygon(const std::shared_ptr<geos::geom::Geometry> & geometry)
{
  std::vector<Plane> res;
  geos::geom::Polygon * polygon = dynamic_cast<geos::geom::Polygon*>(geometry.get());
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
      plane.normal = plane.normal/norm;
    }
    else
    {
      plane.normal = Eigen::Vector3d::Zero();
    }
    plane.offset = -1*(plane.normal.x()*p.x + plane.normal.y()*p.y);
    res.push_back(plane);
  }
  return res;
}

void set_planes(const std::vector<Plane> & planes,
                const std::shared_ptr<tasks::qp::CoMIncPlaneConstr> & constr,
                const std::vector<Eigen::Vector3d> & speeds,
                const std::vector<Eigen::Vector3d> & normalsDots)
{
  constr->reset();
  if(speeds.size() != 0 and normalsDots.size() == speeds.size() and planes.size() == speeds.size())
  {
    for(size_t i = 0; i < planes.size(); ++i)
    {
      constr->addPlane(i, planes[i].normal, planes[i].offset, 0.05, 0.01, 0.1, speeds[i], normalsDots[i], 0.);
    }
  }
  else
  {
    if(speeds.size() != 0 and (normalsDots.size() != speeds.size()
                               or planes.size() != speeds.size()))
    {
      LOG_WARNING("set_planes: speeds size > 0 but different from normalsDots or planes, acting as if speeds were not provided")
    }
    for(size_t i = 0; i < planes.size(); ++i)
    {
      constr->addPlane(i, planes[i].normal, planes[i].offset, 0.04, 0.01, 0.01, 0.);
    }
  }
  constr->updateNrPlanes();
}

}
