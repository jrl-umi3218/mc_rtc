#pragma once

#include <array>
#include <memory>
#include <geos/geom/GeometryFactory.h>
#include <jsoncpp/json/json.h>

namespace mc_rbdyn
{

struct PolygonInterpolator
{
private:
  typedef std::array<double, 2> tuple_t;
  typedef std::pair<tuple_t, tuple_t> tuple_pair_t;

  struct GeometryDeleter
  {
  public:
    GeometryDeleter(geos::geom::GeometryFactory & factory);

    void operator()(geos::geom::Geometry * ptr);
  private:
      geos::geom::GeometryFactory & factory;
  };
public:
  /* For now, the PolygonInterpolator can only be restored from a serialized
   * instance of the Python object */
  PolygonInterpolator(const Json::Value & v);

  std::shared_ptr<geos::geom::Geometry> fast_interpolate(double percent);

  std::vector<tuple_t> midpoint_derivative(double epsilon_derivative);

  std::vector<tuple_t> normal_derivative(double epsilon_derivative);
private:
  geos::geom::GeometryFactory geom_factory;
  GeometryDeleter geom_deleter;
  std::vector<tuple_pair_t> tuple_pairs;
  std::vector<tuple_pair_t> midpoints;
};

}
