#include <mc_rbdyn/Contact.h>

#include <mc_rbdyn/contact_transform.h>
#include <mc_rbdyn/robot.h>

#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/stance.h>

#include <geos/geom/Polygon.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/GeometryFactory.h>

namespace mc_rbdyn
{

struct ContactImpl
{
public:
  unsigned int r1Index;
  unsigned int r2Index;
  std::shared_ptr<mc_rbdyn::Surface> r1Surface;
  std::shared_ptr<mc_rbdyn::Surface> r2Surface;
  sva::PTransformd X_r2s_r1s;
  bool is_fixed;
  sva::PTransformd X_b_s;
  int ambiguityId;
};

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

Contact::Contact(const mc_rbdyn::Robots & robots, const std::string & robotSurface, const std::string & envSurface)
: Contact(robots, robotSurface, envSurface, sva::PTransformd::Identity(), false)
{
}

Contact::Contact(const mc_rbdyn::Robots & robots, const std::string & robotSurface, const std::string & envSurface, const sva::PTransformd & X_es_rs)
: Contact(robots, robotSurface, envSurface, X_es_rs, true)
{
}

Contact::Contact(const mc_rbdyn::Robots & robots, const std::string & robotSurface, const std::string & envSurface, const sva::PTransformd & X_es_rs, bool is_fixed)
{
  if(robots.robots[0].surfaces.count(robotSurface) == 0)
  {
    std::stringstream ss;
    ss << "No surface named " << robotSurface << " in robot" << std::endl;
    std::cerr << ss.str() << std::endl;
    throw(ss.str());
  }
  if(robots.robots[1].surfaces.count(envSurface) == 0)
  {
    std::stringstream ss;
    ss << "No surface named " << envSurface << " in env" << std::endl;
    std::cerr << ss.str() << std::endl;
    throw(ss.str());
  }
  impl.reset(new ContactImpl({0, 1,
    robots.robots[0].surfaces.at(robotSurface)->copy(),
    robots.robots[1].surfaces.at(envSurface)->copy(),
    X_es_rs, is_fixed, sva::PTransformd::Identity(), -1}));
}

Contact::Contact(const mc_rbdyn::Robots & robots, unsigned int r1Index, unsigned int r2Index,
            const std::string & r1Surface, const std::string & r2Surface,
            const sva::PTransformd * X_r2s_r1s,
            const sva::PTransformd & Xbs, int ambiguityId)
{
  if(robots.robots[r1Index].surfaces.count(r1Surface) == 0)
  {
    std::stringstream ss;
    ss << "No surfaces named " << r1Surface << " in robot " << r1Index << std::endl;
    std::cerr << ss.str() << std::endl;
    throw(ss.str());
  }
  if(robots.robots[r2Index].surfaces.count(r2Surface) == 0)
  {
    std::stringstream ss;
    ss << "No surfaces named " << r2Surface << " in robot " << r2Index << std::endl;
    std::cerr << ss.str() << std::endl;
    throw(ss.str());
  }
  impl.reset(new ContactImpl({r1Index, r2Index,
    robots.robots[r1Index].surfaces.at(r1Surface)->copy(),
    robots.robots[r2Index].surfaces.at(r2Surface)->copy(),
    sva::PTransformd::Identity(), X_r2s_r1s != 0, Xbs, ambiguityId}));
  if(isFixed())
  {
    impl->X_r2s_r1s = sva::PTransformd(*X_r2s_r1s);
  }
}

Contact::Contact(const Contact & contact)
{
  impl.reset(new ContactImpl({contact.r1Index(), contact.r2Index(),
    contact.r1Surface()->copy(), contact.r2Surface()->copy(),
    contact.X_r2s_r1s(), contact.isFixed(),
    contact.X_b_s(), contact.ambiguityId()}));
}

Contact & Contact::operator=(const Contact & rhs)
{
  if(this == &rhs) { return *this; }
  this->impl->r1Index = rhs.r1Index();
  this->impl->r2Index = rhs.r2Index();
  this->impl->r1Surface = rhs.r1Surface()->copy();
  this->impl->r2Surface = rhs.r2Surface()->copy();
  this->impl->X_r2s_r1s = rhs.X_r2s_r1s();
  this->impl->is_fixed = rhs.isFixed();
  this->impl->X_b_s = rhs.X_b_s();
  this->impl->ambiguityId = rhs.ambiguityId();
  return *this;
}

Contact::~Contact()
{
}

unsigned int Contact::r1Index() const
{
  return impl->r1Index;
}

unsigned int Contact::r2Index() const
{
  return impl->r2Index;
}

const std::shared_ptr<mc_rbdyn::Surface> & Contact::r1Surface() const
{
  return impl->r1Surface;
}

const std::shared_ptr<mc_rbdyn::Surface> & Contact::r2Surface() const
{
  return impl->r2Surface;
}

const sva::PTransformd & Contact::X_r2s_r1s() const
{
  return impl->X_r2s_r1s;
}

void Contact::X_r2s_r1s(const sva::PTransformd & in)
{
  impl->X_r2s_r1s = in;
}

const sva::PTransformd & Contact::X_b_s() const
{
  return impl->X_b_s;
}

const int & Contact::ambiguityId() const
{
  return impl->ambiguityId;
}

bool Contact::isFixed() const
{
  return impl->is_fixed;
}

std::pair<std::string, std::string> Contact::surfaces() const
{
  return std::pair<std::string, std::string>(impl->r1Surface->name(), impl->r2Surface->name());
}

sva::PTransformd Contact::X_0_r1s(const mc_rbdyn::Robots & robots) const
{
  return impl->X_r2s_r1s*(impl->r2Surface->X_0_s(robots.robots[impl->r2Index]));
}

sva::PTransformd Contact::X_0_r2s(const mc_rbdyn::Robots & robots) const
{
  return impl->X_r2s_r1s.inv()*(impl->r1Surface->X_0_s(robots.robots[impl->r1Index]));
}

std::vector<sva::PTransformd> Contact::r1Points()
{
  if(isFixed())
  {
    return computePoints(*(r1Surface()), *(r2Surface()), X_r2s_r1s());
  }
  else
  {
    const auto & s = *(r1Surface());
    return s.points();
  }
}

std::vector<sva::PTransformd> Contact::r2Points()
{
  if(isFixed())
  {
    return computePoints(*(r2Surface()), *(r1Surface()), X_r2s_r1s().inv());
  }
  else
  {
    const auto & s = *(r2Surface());
    return s.points();
  }
}

sva::PTransformd Contact::compute_X_r2s_r1s(const mc_rbdyn::Robots & robots) const
{
  sva::PTransformd X_0_r1 = impl->r1Surface->X_0_s(robots.robots[impl->r1Index]);
  sva::PTransformd X_0_r2 = impl->r2Surface->X_0_s(robots.robots[impl->r2Index]);
  return X_0_r1*X_0_r2.inv();
}

tasks::qp::ContactId Contact::contactId(const mc_rbdyn::Robots & robots) const
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
  return tasks::qp::ContactId(impl->r1Index, impl->r2Index,
                              robots.robots[impl->r1Index].bodyIdByName(impl->r1Surface->bodyName()),
                              robots.robots[impl->r2Index].bodyIdByName(impl->r2Surface->bodyName()));
#pragma GCC diagnostic pop
}

mc_solver::QPContactPtr Contact::taskContact(const mc_rbdyn::Robots & robots) const
{
  const mc_rbdyn::Robot & r1 = robots.robots[impl->r1Index];
  const mc_rbdyn::Robot & r2 = robots.robots[impl->r2Index];
  unsigned int r1BodyIndex = r1.bodyIndexByName(impl->r1Surface->bodyName());
  unsigned int r2BodyIndex = r2.bodyIndexByName(impl->r2Surface->bodyName());
  sva::PTransformd X_0_b1 = r1.mbc->bodyPosW[r1BodyIndex];
  sva::PTransformd X_0_b2 = r2.mbc->bodyPosW[r2BodyIndex];
  sva::PTransformd X_b1_b2 = X_0_b2*X_0_b1.inv();
  const auto & r1Surface = *(impl->r1Surface);
  return taskContact(robots, X_b1_b2, r1Surface.points());
}

mc_solver::QPContactPtrWPoints Contact::taskContactWPoints(const mc_rbdyn::Robots & robots, const sva::PTransformd * X_es_rs) const
{
  mc_solver::QPContactPtrWPoints res;
  if(X_es_rs)
  {
    sva::PTransformd X_b1_b2 = X_es_rs->inv();
    res.points = computePoints(*(impl->r1Surface), *(impl->r2Surface), *X_es_rs);
    res.qpcontact_ptr = taskContact(robots, X_b1_b2, res.points);
  }
  else
  {
    res.qpcontact_ptr = taskContact(robots);
    const auto & r1Surface = *(impl->r1Surface);
    res.points = r1Surface.points();
  }
  return res;
}

mc_solver::QPContactPtr Contact::taskContact(const mc_rbdyn::Robots & robots, const sva::PTransformd & X_b1_b2, const std::vector<sva::PTransformd> & surf_points) const
{
  mc_solver::QPContactPtr res;

  const mc_rbdyn::Robot & r1 = robots.robots[impl->r1Index];
  const mc_rbdyn::Robot & r2 = robots.robots[impl->r2Index];

  unsigned int r1BodyId = r1.bodyIdByName(impl->r1Surface->bodyName());
  unsigned int r2BodyId = r2.bodyIdByName(impl->r2Surface->bodyName());

  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Matrix3d> frames;
  for(const auto & p : surf_points)
  {
    points.push_back(p.translation());
    frames.push_back(p.rotation());
  }

  if(impl->r1Surface->type() == "planar")
  {
    res.unilateralContact = new tasks::qp::UnilateralContact(impl->r1Index, impl->r2Index, r1BodyId, r2BodyId, impl->ambiguityId, points, frames[0], X_b1_b2, Stance::nrConeGen, Stance::defaultFriction, impl->X_b_s);
  }
  else if(impl->r1Surface->type() == "gripper")
  {
    res.bilateralContact = new tasks::qp::BilateralContact(impl->r1Index, impl->r2Index, r1BodyId, r2BodyId, impl->ambiguityId, points, frames, X_b1_b2, Stance::nrConeGen, Stance::defaultFriction, impl->X_b_s);
  }
  else
  {
    std::string err = "Robot's contact surface is neither planar nor gripper";
    std::cerr << err << std::endl;
    throw(err.c_str());
  }

  return res;
}

std::string Contact::toStr() const
{
  std::stringstream ss;
  ss << impl->r1Surface->toStr() << "/" << impl->r2Surface->toStr();
  return ss.str();
}

bool operator==(const Contact & lhs, const Contact & rhs)
{
  return (*(lhs.r1Surface()) == *(rhs.r1Surface())) and (*(lhs.r2Surface()) == *(rhs.r2Surface()));
}

bool operator!=(const Contact & lhs, const Contact & rhs)
{
  return not(lhs == rhs);
}

}
