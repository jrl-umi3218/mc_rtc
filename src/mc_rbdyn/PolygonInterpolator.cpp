/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/PolygonInterpolator.h>

#include <geos/version.h>

#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/Polygon.h>

namespace mc_rbdyn
{

PolygonInterpolator::GeometryDeleter::GeometryDeleter(const geos::geom::GeometryFactory & factory) : factory(factory) {}

void PolygonInterpolator::GeometryDeleter::operator()(geos::geom::Geometry * ptr)
{
  factory.destroyGeometry(ptr);
}

PolygonInterpolator::PolygonInterpolator(const std::vector<tuple_pair_t> & tpv)
: geom_factory(*geos::geom::GeometryFactory::getDefaultInstance()), geom_deleter(geom_factory), tuple_pairs_(tpv)
{
  for(size_t i = 0; i < tuple_pairs_.size(); ++i)
  {
    const tuple_pair_t & prev = (i == 0 ? tuple_pairs_.back() : tuple_pairs_[i - 1]);
    const tuple_t & prev_1 = prev.first;
    const tuple_t & prev_2 = prev.second;
    const tuple_t & point_1 = tuple_pairs_[i].first;
    const tuple_t & point_2 = tuple_pairs_[i].second;
    midpoints.push_back({{{(point_1[0] + prev_1[0]) / 2, (point_1[1] + prev_1[1]) / 2}},
                         {{(point_2[0] + prev_2[0]) / 2, (point_2[1] + prev_2[1]) / 2}}});
  }
}

std::shared_ptr<geos::geom::Geometry> PolygonInterpolator::fast_interpolate(double percent)
{
  double perc = std::max(std::min(percent, 1.), -1.);
  if(perc < 0)
  {
    perc = 1 + perc;
  }
  std::vector<tuple_t> points;
  auto seq = geom_factory.getCoordinateSequenceFactory()->create(static_cast<size_t>(0), 0);
  std::vector<geos::geom::Coordinate> seq_points;
  for(const auto & p : tuple_pairs_)
  {
    seq_points.push_back(geos::geom::Coordinate(static_cast<float>(p.first[0] * (1 - perc) + p.second[0] * perc),
                                                static_cast<float>(p.first[1] * (1 - perc) + p.second[1] * perc)));
  }
  seq_points.push_back(seq_points[0]);
  seq->setPoints(seq_points);
  auto shell = geom_factory.createLinearRing(std::move(seq));
#if GEOS_VERSION_MAJOR >= 3 && GEOS_VERSION_MINOR >= 8
  auto poly = geom_factory.createPolygon(std::move(shell));
  std::shared_ptr<geos::geom::Geometry> ret(poly->convexHull().release(), geom_deleter);
  geom_factory.destroyGeometry(poly.release());
#else
  geos::geom::Polygon * poly = geom_factory.createPolygon(shell, 0);
  std::shared_ptr<geos::geom::Geometry> ret(poly->convexHull(), geom_deleter);
  geom_factory.destroyGeometry(poly);
#endif
  return ret;
}

std::vector<PolygonInterpolator::tuple_t> PolygonInterpolator::midpoint_derivative(double epsilon_derivative)
{
  std::vector<tuple_t> res;
  for(const auto & p : midpoints)
  {
    res.push_back({{(p.second[0] - p.first[0]) / epsilon_derivative, (p.second[1] - p.first[1]) / epsilon_derivative}});
  }
  return res;
}

std::vector<PolygonInterpolator::tuple_t> PolygonInterpolator::normal_derivative(double epsilon_derivative)
{
  std::vector<tuple_t> res;
  auto seq_s = geom_factory.getCoordinateSequenceFactory()->create(static_cast<size_t>(0), 2);
  auto seq_d = geom_factory.getCoordinateSequenceFactory()->create(static_cast<size_t>(0), 2);
  std::vector<geos::geom::Coordinate> points_s;
  std::vector<geos::geom::Coordinate> points_d;
  for(const auto & p : tuple_pairs_)
  {
    points_s.push_back(geos::geom::Coordinate(static_cast<float>(p.first[0]), static_cast<float>(p.first[1])));
    points_d.push_back(geos::geom::Coordinate(static_cast<float>(p.second[0]), static_cast<float>(p.second[1])));
  }
  points_s.push_back(points_s[0]);
  points_d.push_back(points_d[0]);
  seq_s->setPoints(points_s);
  seq_d->setPoints(points_d);
  auto shell_s = geom_factory.createLinearRing(std::move(seq_s));
  auto shell_d = geom_factory.createLinearRing(std::move(seq_d));
#if GEOS_VERSION_MAJOR >= 3 && GEOS_VERSION_MINOR >= 8
  auto poly_s = geom_factory.createPolygon(std::move(shell_s));
  auto poly_d = geom_factory.createPolygon(std::move(shell_d));
#else
  geos::geom::Polygon * poly_s = geom_factory.createPolygon(shell_s, 0);
  geos::geom::Polygon * poly_d = geom_factory.createPolygon(shell_d, 0);
#endif
#if GEOS_VERSION_MAJOR >= 3 && GEOS_VERSION_MINOR >= 8
  auto normals = [](std::unique_ptr<geos::geom::Polygon> & poly) {
#else
  auto normals = [](geos::geom::Polygon * poly) {
#endif
    std::vector<tuple_t> _res;
    auto seq = poly->getExteriorRing()->getCoordinates();
    for(size_t i = 0; i < seq->size() - 1; ++i)
    {
      const geos::geom::Coordinate & p = seq->getAt(i);
      const geos::geom::Coordinate & prev = seq->getAt(i == 0 ? seq->size() - 1 : i - 1);
      tuple_t normal{{p.y - prev.y, -(p.x - prev.x)}};
      double norm = normal[0] * normal[0] + normal[1] * normal[1];
      if(norm > 0)
      {
        _res.push_back({{normal[0] / norm, normal[1] / norm}});
      }
      else
      {
        _res.push_back({{0., 0.}});
      }
    }
    return _res;
  };
  auto n_strt = normals(poly_s);
  auto n_dest = normals(poly_d);
#if GEOS_VERSION_MAJOR >= 3 && GEOS_VERSION_MINOR >= 8
#else
  geom_factory.destroyGeometry(poly_s);
  geom_factory.destroyGeometry(poly_d);
#endif
  for(size_t i = 0; i < std::min(n_strt.size(), n_dest.size()); ++i)
  {
    if(n_strt[i][0] == 0 && n_strt[i][1] == 0 && n_dest[i][0] == 0 && n_dest[i][1] == 0)
    {
      res.push_back({{0., 0.}});
    }
    else
    {
      res.push_back(
          {{(n_dest[i][0] - n_strt[i][0]) / epsilon_derivative, (n_dest[i][1] - n_strt[i][1]) / epsilon_derivative}});
    }
  }
  return res;
}

const std::vector<PolygonInterpolator::tuple_pair_t> & PolygonInterpolator::tuple_pairs() const
{
  return tuple_pairs_;
}

} // namespace mc_rbdyn
