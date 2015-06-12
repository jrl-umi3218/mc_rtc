#include <mc_rbdyn/contact.h>

#include <mc_rbdyn/contact_transform.h>
#include <mc_rbdyn/robot.h>

#include <mc_rbdyn/PlanarSurface.h>

#include <geos/geom/Polygon.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/GeometryFactory.h>

namespace mc_rbdyn
{

std::vector<sva::PTransformd> computePoints(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface, const sva::PTransformd & X_es_rs)
{
  if(robotSurface.type() == "gripper")
  {
    return robotSurface.points();
  }
  if( (envSurface.type() == "planar" or envSurface.type() == "cylindrical") and robotSurface.type() == "planar" )
  {
    // Transform env points in robot surface coordinate
    std::vector<sva::PTransformd> envPointsInRobotSurface(0);
    for(const sva::PTransformd & p : envSurface.points())
    {
      envPointsInRobotSurface.push_back(p*envSurface.X_b_s().inv()*X_es_rs.inv());
    }

    // Project robot and env points in robot surface in 2d
    Eigen::Vector3d robotT = robotSurface.X_b_s().rotation().row(0).transpose();
    Eigen::Vector3d robotB = robotSurface.X_b_s().rotation().row(1).transpose();
    auto proj2D = [robotT, robotB](const sva::PTransformd & p)
                  { return std::pair<double, double>(robotT.dot(p.translation()), robotB.dot(p.translation())); };
    std::vector< std::pair<double, double> > envPoints2d(0);
    for(const sva::PTransformd & p : envPointsInRobotSurface)
    {
      envPoints2d.push_back(proj2D(p));
    }
    const std::vector< std::pair<double, double> > & robotPoints2d = (reinterpret_cast<const PlanarSurface&>(robotSurface)).planarPoints();

    //Compute the intersection
    geos::geom::GeometryFactory factory;

    // Create robot surf polygon
    geos::geom::CoordinateSequence * robotPoints2dseq = factory.getCoordinateSequenceFactory()->create((std::size_t)0,0);
    for(const std::pair<double, double> & p : robotPoints2d)
    {
      robotPoints2dseq->add(geos::geom::Coordinate(p.first, p.second));
    }
    robotPoints2dseq->add(geos::geom::Coordinate(robotPoints2d[0].first, robotPoints2d[0].second));
    geos::geom::LinearRing * robotPoints2dshell = factory.createLinearRing(robotPoints2dseq);
    geos::geom::Polygon * robotSurfPoly = factory.createPolygon(robotPoints2dshell, 0);

    // Create env surf polygon
    geos::geom::CoordinateSequence * envPoints2dseq = factory.getCoordinateSequenceFactory()->create((std::size_t)0,0);
    for(const std::pair<double, double> & p : envPoints2d)
    {
      envPoints2dseq->add(geos::geom::Coordinate(p.first, p.second));
    }
    envPoints2dseq->add(geos::geom::Coordinate(envPoints2d[0].first, envPoints2d[0].second));
    geos::geom::LinearRing * envPoints2dshell = factory.createLinearRing(envPoints2dseq);
    geos::geom::Polygon * envSurfPoly = factory.createPolygon(envPoints2dshell, 0);

    geos::geom::Geometry * newRobotSurfGeom = robotSurfPoly->intersection(envSurfPoly);
    geos::geom::Polygon * newRobotSurfPoly = dynamic_cast<geos::geom::Polygon*>(newRobotSurfGeom);
    if(newRobotSurfPoly == 0)
    {
      std::cout << robotSurface.name() << " and " << envSurface.name() << " surfaces don't intersect" << std::endl;
      return robotSurface.points();
    }
    std::vector<sva::PTransformd> res;
    const geos::geom::CoordinateSequence * newPoints = newRobotSurfPoly->getExteriorRing()->getCoordinates();
    for(size_t i = 0; i < newPoints->getSize() - 1; ++i)
    {
      const geos::geom::Coordinate & p = newPoints->getAt(i);
      res.push_back(sva::PTransformd(Eigen::Vector3d(p.x, p.y, 0))*robotSurface.X_b_s());
    }
    return res;
  }
  std::cerr << "Surfaces " << robotSurface.name() << " and " << envSurface.name() << " have incompatible types for contact" << std::endl;
  throw(std::string("type error"));
}

Contact::Contact(const std::shared_ptr<mc_rbdyn::Surface> & robotSurface, const std::shared_ptr<mc_rbdyn::Surface> & envSurface)
: Contact(*robotSurface, *envSurface, sva::PTransformd::Identity(), false)
{
}

Contact::Contact(const std::shared_ptr<mc_rbdyn::Surface> & robotSurface, const std::shared_ptr<mc_rbdyn::Surface> & envSurface, const sva::PTransformd & X_es_rs)
: Contact(*robotSurface, *envSurface, X_es_rs, true)
{
}

Contact::Contact(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface)
: Contact(robotSurface, envSurface, sva::PTransformd::Identity(), false)
{
}

Contact::Contact(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface, const sva::PTransformd & X_es_rs)
: Contact(robotSurface, envSurface, X_es_rs, true)
{
}

Contact::Contact(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface, const sva::PTransformd & X_es_rs, bool is_fixed)
: r1Index(0), r2Index(1),
  r1Surface(robotSurface.copy()), r2Surface(envSurface.copy()),
  X_r2s_r1s(X_es_rs), is_fixed(is_fixed)
{
}

Contact::Contact(unsigned int r1Index, unsigned int r2Index,
            const std::shared_ptr<mc_rbdyn::Surface> & r1Surface,
            const std::shared_ptr<mc_rbdyn::Surface> & r2Surface,
            const sva::PTransformd * X_r2s_r1s,
            const sva::PTransformd & Xbs, int ambiguityId)
: r1Index(r1Index), r2Index(r2Index),
  r1Surface(r1Surface->copy()), r2Surface(r2Surface->copy()),
  X_r2s_r1s(), is_fixed(X_r2s_r1s != 0), X_b_s(Xbs),
  ambiguityId(ambiguityId)
{
  if(is_fixed)
  {
    this->X_r2s_r1s = sva::PTransformd(*X_r2s_r1s);
  }
}

Contact::Contact(const Contact & contact)
: Contact(*(contact.r1Surface), *(contact.r2Surface), contact.X_r2s_r1s, contact.is_fixed)
{
}

Contact & Contact::operator=(const Contact & rhs)
{
  if(this == &rhs) { return *this; }
  this->r1Surface = rhs.r1Surface->copy();
  this->r2Surface = rhs.r2Surface->copy();
  this->X_r2s_r1s = rhs.X_r2s_r1s;
  this->is_fixed = rhs.is_fixed;
  this->X_b_s = rhs.X_b_s;
  return *this;
}

bool Contact::isFixed()
{
  return is_fixed;
}

std::pair<std::string, std::string> Contact::surfaces() const
{
  return std::pair<std::string, std::string>(r1Surface->name(), r2Surface->name());
}

sva::PTransformd Contact::X_0_r1s(const mc_rbdyn::Robots & robots) const
{
  return X_r2s_r1s*(r2Surface->X_0_s(robots.robots[r2Index]));
}

sva::PTransformd Contact::X_0_r2s(const mc_rbdyn::Robots & robots) const
{
  return X_r2s_r1s.inv()*(r1Surface->X_0_s(robots.robots[r1Index]));
}

std::vector<sva::PTransformd> Contact::r1Points()
{
  if(isFixed())
  {
    return computePoints(*r1Surface, *r2Surface, X_r2s_r1s);
  }
  else
  {
    const auto & s = *r1Surface;
    return s.points();
  }
}

std::vector<sva::PTransformd> Contact::r2Points()
{
  if(isFixed())
  {
    return computePoints(*r2Surface, *r1Surface, X_r2s_r1s.inv());
  }
  else
  {
    const auto & s = *r2Surface;
    return s.points();
  }
}

sva::PTransformd Contact::compute_X_r2s_r1s(const mc_rbdyn::Robots & robots) const
{
  sva::PTransformd X_0_r1 = r1Surface->X_0_s(robots.robots[r1Index]);
  sva::PTransformd X_0_r2 = r2Surface->X_0_s(robots.robots[r2Index]);
  return X_0_r1*X_0_r2.inv();
}

tasks::qp::ContactId Contact::contactId(const mc_rbdyn::Robots & robots) const
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
  return tasks::qp::ContactId(r1Index, r2Index,
                              robots.robots[r1Index].bodyIdByName(r1Surface->bodyName()),
                              robots.robots[r2Index].bodyIdByName(r2Surface->bodyName()));
#pragma GCC diagnostic pop
}

std::string Contact::toStr() const
{
  std::stringstream ss;
  ss << r1Surface->toStr() << "/" << r2Surface->toStr();
  return ss.str();
}

bool operator==(const Contact & lhs, const Contact & rhs)
{
  return (*(lhs.r1Surface) == *(rhs.r1Surface)) and (*(lhs.r2Surface) == *(rhs.r2Surface));
}

bool operator!=(const Contact & lhs, const Contact & rhs)
{
  return not(lhs == rhs);
}

}
