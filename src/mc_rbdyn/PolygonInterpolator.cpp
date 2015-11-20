#include <mc_rbdyn/PolygonInterpolator.h>

#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/Polygon.h>

namespace mc_rbdyn
{

PolygonInterpolator::GeometryDeleter::GeometryDeleter(geos::geom::GeometryFactory & factory)
: factory(factory)
{
}

void PolygonInterpolator::GeometryDeleter::operator()(geos::geom::Geometry * ptr)
{
  factory.destroyGeometry(ptr);
}

PolygonInterpolator::PolygonInterpolator(const Json::Value & jsv)
: geom_factory(), geom_deleter(geom_factory), tuple_pairs()
{
  if(jsv.isMember("tuple_pairs"))
  {
    for(const auto & tpv : jsv["tuple_pairs"])
    {
      if(tpv.isMember("p1") and tpv.isMember("p2"))
      {
        const auto & p1 = tpv["p1"];
        const auto & p2 = tpv["p2"];
        if(p1.isArray() and p1.size() == 2 and
           p2.isArray() and p2.size() == 2)
        {
          tuple_pairs.push_back({
            {{p1[0].asDouble(), p1[1].asDouble()}},
            {{p2[0].asDouble(), p2[1].asDouble()}}
          });
        }
      }
    }
  }
  for(size_t i = 0; i < tuple_pairs.size(); ++i)
  {
    const tuple_pair_t & prev = (i == 0 ? tuple_pairs.back() : tuple_pairs[i-1]);
    const tuple_t & prev_1 = prev.first;
    const tuple_t & prev_2 = prev.second;
    const tuple_t & point_1 = tuple_pairs[i].first;
    const tuple_t & point_2 = tuple_pairs[i].second;
    midpoints.push_back({
      {{ (point_1[0] + prev_1[0])/2, (point_1[1] + prev_1[1])/2 }},
      {{ (point_2[0] + prev_2[0])/2, (point_2[1] + prev_2[1])/2 }}
      });
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
  geos::geom::CoordinateSequence * seq = geom_factory.getCoordinateSequenceFactory()->create((std::size_t)0, 0);
  for(const auto & p : tuple_pairs)
  {
    seq->add(geos::geom::Coordinate(p.first[0]*(1-perc) + p.second[0]*perc,
                                    p.first[1]*(1-perc) + p.second[1]*perc));
  }
  seq->add(seq->getAt(0));
  geos::geom::LinearRing * shell = geom_factory.createLinearRing(seq);
  geos::geom::Polygon * poly = geom_factory.createPolygon(shell, 0);
  std::shared_ptr<geos::geom::Geometry> ret(poly->convexHull(), geom_deleter);
  geom_factory.destroyGeometry(poly);
  return ret;
}

std::vector<PolygonInterpolator::tuple_t> PolygonInterpolator::midpoint_derivative(double epsilon_derivative)
{
  std::vector<tuple_t> res;
  for(const auto & p : midpoints)
  {
    res.push_back(
      {{(p.second[0] - p.first[0])/epsilon_derivative,
      (p.second[1] - p.first[1])/epsilon_derivative}}
    );
  }
  return res;
}

std::vector<PolygonInterpolator::tuple_t> PolygonInterpolator::normal_derivative(double epsilon_derivative)
{
  std::vector<tuple_t> res;
  geos::geom::CoordinateSequence * seq_s = geom_factory.getCoordinateSequenceFactory()->create((std::size_t)0, 2);
  geos::geom::CoordinateSequence * seq_d = geom_factory.getCoordinateSequenceFactory()->create((std::size_t)0, 2);
  for(const auto & p : tuple_pairs)
  {
    seq_s->add(geos::geom::Coordinate(p.first[0], p.first[1]));
    seq_d->add(geos::geom::Coordinate(p.second[0], p.second[1]));
  }
  seq_s->add(seq_s->getAt(0));
  seq_d->add(seq_d->getAt(0));
  geos::geom::LinearRing * shell_s = geom_factory.createLinearRing(seq_s);
  geos::geom::LinearRing * shell_d = geom_factory.createLinearRing(seq_d);
  geos::geom::Polygon * poly_s = geom_factory.createPolygon(shell_s, 0);
  geos::geom::Polygon * poly_d = geom_factory.createPolygon(shell_d, 0);
  auto normals = [](geos::geom::Polygon * poly)
  {
    std::vector<tuple_t> _res;
    const geos::geom::CoordinateSequence * seq = poly->getExteriorRing()->getCoordinates();
    for(size_t i = 0; i < seq->size() - 1; ++i)
    {
      const geos::geom::Coordinate & p = seq->getAt(i);
      const geos::geom::Coordinate & prev = seq->getAt(i == 0 ? seq->size() - 1 : i - 1);
      tuple_t normal {{p.y - prev.y, -(p.x - prev.x)}};
      double norm = normal[0]*normal[0] + normal[1]*normal[1];
      if(norm > 0)
      {
        _res.push_back({{normal[0]/norm, normal[1]/norm}});
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
  geom_factory.destroyGeometry(poly_s);
  geom_factory.destroyGeometry(poly_d);
  for(size_t i = 0; i < std::min(n_strt.size(), n_dest.size()); ++i)
  {
    if(n_strt[i][0] == 0 and n_strt[i][1] == 0 and n_dest[i][0] == 0 and n_dest[i][1] == 0)
    {
      res.push_back({{0.,0.}});
    }
    else
    {
      res.push_back({{
        (n_dest[i][0] - n_strt[i][0])/epsilon_derivative,
        (n_dest[i][1] - n_strt[i][1])/epsilon_derivative
      }});
    }
  }
  return res;
}

}
