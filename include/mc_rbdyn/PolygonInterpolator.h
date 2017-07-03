#pragma once

#include <array>
#include <memory>
#include <geos/geom/GeometryFactory.h>

#include <mc_rbdyn/api.h>

namespace mc_rbdyn
{

struct MC_RBDYN_DLLAPI PolygonInterpolator
{
public:
  typedef std::array<double, 2> tuple_t;
  typedef std::pair<tuple_t, tuple_t> tuple_pair_t;
private:
  struct GeometryDeleter
  {
  public:
    GeometryDeleter(const geos::geom::GeometryFactory & factory);

    void operator()(geos::geom::Geometry * ptr);
  private:
      const geos::geom::GeometryFactory & factory;
  };
public:
  PolygonInterpolator(const std::vector<tuple_pair_t> & tpv);

  std::shared_ptr<geos::geom::Geometry> fast_interpolate(double percent);

  std::vector<tuple_t> midpoint_derivative(double epsilon_derivative);

  std::vector<tuple_t> normal_derivative(double epsilon_derivative);

  const std::vector<tuple_pair_t> & tuple_pairs() const;
private:
  const geos::geom::GeometryFactory & geom_factory;
  GeometryDeleter geom_deleter;
  std::vector<tuple_pair_t> tuple_pairs_;
  std::vector<tuple_pair_t> midpoints;
};

}
