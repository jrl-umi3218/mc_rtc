/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/contact_transform.h>
#include <mc_rtc/logging.h>

#include <geos/version.h>

#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/Polygon.h>

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

std::vector<sva::PTransformd> computePoints(const mc_rbdyn::Surface & robotSurface,
                                            const mc_rbdyn::Surface & envSurface,
                                            const sva::PTransformd & X_es_rs)
{
  if(robotSurface.type() == "gripper")
  {
    return robotSurface.points();
  }
  if((envSurface.type() == "planar" || envSurface.type() == "cylindrical") && robotSurface.type() == "planar")
  {
    // Transform env points in robot surface coordinate
    std::vector<sva::PTransformd> envPointsInRobotSurface(0);
    for(const sva::PTransformd & p : envSurface.points())
    {
      envPointsInRobotSurface.push_back(p * envSurface.X_b_s().inv() * X_es_rs.inv());
    }

    // Project robot and env points in robot surface in 2d
    Eigen::Vector3d robotT = robotSurface.X_b_s().rotation().row(0).transpose();
    Eigen::Vector3d robotB = robotSurface.X_b_s().rotation().row(1).transpose();
    auto proj2D = [robotT, robotB](const sva::PTransformd & p) {
      return std::pair<double, double>(robotT.dot(p.translation()), robotB.dot(p.translation()));
    };
    std::vector<std::pair<double, double>> envPoints2d(0);
    for(const sva::PTransformd & p : envPointsInRobotSurface)
    {
      envPoints2d.push_back(proj2D(p));
    }
    const std::vector<std::pair<double, double>> & robotPoints2d =
        (reinterpret_cast<const PlanarSurface &>(robotSurface)).planarPoints();

    // Compute the intersection
    const geos::geom::GeometryFactory * factory_ptr = geos::geom::GeometryFactory::getDefaultInstance();
    const geos::geom::GeometryFactory & factory = *factory_ptr;

    // Create robot surf polygon
    auto robotPoints2dseq = factory.getCoordinateSequenceFactory()->create(static_cast<size_t>(0), 0);
    std::vector<geos::geom::Coordinate> points;
    for(const std::pair<double, double> & p : robotPoints2d)
    {
      points.push_back(geos::geom::Coordinate(p.first, p.second));
    }
    points.push_back(geos::geom::Coordinate(robotPoints2d[0].first, robotPoints2d[0].second));
    robotPoints2dseq->setPoints(points);
    auto robotPoints2dshell = factory.createLinearRing(std::move(robotPoints2dseq));
#if GEOS_VERSION_MAJOR >= 3 && GEOS_VERSION_MINOR >= 8
    auto robotSurfPoly = factory.createPolygon(std::move(robotPoints2dshell));
#else
    auto robotSurfPoly = factory.createPolygon(std::move(robotPoints2dshell), nullptr);
#endif

    // Create env surf polygon
    auto envPoints2dseq = factory.getCoordinateSequenceFactory()->create(static_cast<size_t>(0), 0);
    points.clear();
    for(const std::pair<double, double> & p : envPoints2d)
    {
      points.push_back(geos::geom::Coordinate(p.first, p.second));
    }
    points.push_back(geos::geom::Coordinate(envPoints2d[0].first, envPoints2d[0].second));
    envPoints2dseq->setPoints(points);
    auto envPoints2dshell = factory.createLinearRing(std::move(envPoints2dseq));
#if GEOS_VERSION_MAJOR >= 3 && GEOS_VERSION_MINOR >= 8
    auto envSurfPoly = factory.createPolygon(std::move(envPoints2dshell));
    auto newRobotSurfGeom = robotSurfPoly->intersection(envSurfPoly.get());
    auto newRobotSurfPoly = dynamic_cast<geos::geom::Polygon *>(newRobotSurfGeom.get());
#else
    auto envSurfPoly = factory.createPolygon(std::move(envPoints2dshell), nullptr);
    auto newRobotSurfGeom = robotSurfPoly->intersection(envSurfPoly);
    auto newRobotSurfPoly = dynamic_cast<geos::geom::Polygon *>(newRobotSurfGeom);
#endif

    if(newRobotSurfPoly == 0)
    {
      LOG_INFO(robotSurface.name() << " and " << envSurface.name() << " surfaces don't intersect")
      return robotSurface.points();
    }
    std::vector<sva::PTransformd> res;
    auto newPoints = newRobotSurfPoly->getExteriorRing()->getCoordinates();
    for(size_t i = 0; i < newPoints->getSize() - 1; ++i)
    {
      const geos::geom::Coordinate & p = newPoints->getAt(i);
      res.push_back(sva::PTransformd(Eigen::Vector3d(p.x, p.y, 0)) * robotSurface.X_b_s());
    }
    return res;
  }
  LOG_ERROR_AND_THROW(std::runtime_error, "Surfaces " << robotSurface.name() << " and " << envSurface.name()
                                                      << " have incompatible types for contact")
}

Contact::Contact(const mc_rbdyn::Robots & robots, const std::string & robotSurface, const std::string & envSurface)
: Contact(robots, robotSurface, envSurface, sva::PTransformd::Identity(), false)
{
}

Contact::Contact(const mc_rbdyn::Robots & robots,
                 const std::string & robotSurface,
                 const std::string & envSurface,
                 const sva::PTransformd & X_es_rs)
: Contact(robots, robotSurface, envSurface, X_es_rs, true)
{
}

Contact::Contact(const mc_rbdyn::Robots & robots,
                 const std::string & robotSurface,
                 const std::string & envSurface,
                 const sva::PTransformd & X_es_rs,
                 bool is_fixed)
{
  impl.reset(new ContactImpl{0, 1, robots.robot(0).surface(robotSurface).copy(),
                             robots.robot(1).surface(envSurface).copy(), X_es_rs, is_fixed,
                             robots.robot(0).surface(robotSurface).X_b_s(), -1});
}

Contact::Contact(const mc_rbdyn::Robots & robots,
                 unsigned int r1Index,
                 unsigned int r2Index,
                 const std::string & r1Surface,
                 const std::string & r2Surface,
                 int ambiguityId)
: Contact(robots,
          r1Index,
          r2Index,
          r1Surface,
          r2Surface,
          sva::PTransformd::Identity(),
          robots.robot(r1Index).surface(r1Surface).X_b_s(),
          ambiguityId)
{
}

Contact::Contact(const mc_rbdyn::Robots & robots,
                 unsigned int r1Index,
                 unsigned int r2Index,
                 const std::string & r1Surface,
                 const std::string & r2Surface,
                 const sva::PTransformd & X_r2s_r1s,
                 int ambiguityId)
: Contact(robots,
          r1Index,
          r2Index,
          r1Surface,
          r2Surface,
          X_r2s_r1s,
          robots.robot(r1Index).surface(r1Surface).X_b_s(),
          ambiguityId)
{
}

Contact::Contact(const mc_rbdyn::Robots & robots,
                 unsigned int r1Index,
                 unsigned int r2Index,
                 const std::string & r1Surface,
                 const std::string & r2Surface,
                 const sva::PTransformd & X_r2s_r1s,
                 const sva::PTransformd & X_b_s,
                 int ambiguityId)
{
  impl.reset(new ContactImpl{r1Index, r2Index, robots.robot(r1Index).surface(r1Surface).copy(),
                             robots.robot(r2Index).surface(r2Surface).copy(), X_r2s_r1s, true, X_b_s, ambiguityId});
}

mc_rbdyn::Contact Contact::load(const mc_rbdyn::Robots & robots, const mc_rtc::Configuration & config)
{
  return mc_rtc::ConfigurationLoader<mc_rbdyn::Contact>::load(config, robots);
}

std::vector<mc_rbdyn::Contact> Contact::loadVector(const mc_rbdyn::Robots & robots,
                                                   const mc_rtc::Configuration & config)
{
  std::vector<mc_rbdyn::Contact> ret;
  for(const auto & c : config)
  {
    ret.emplace_back(load(robots, c));
  }
  return ret;
}

Contact::Contact(const Contact & contact)
{
  impl.reset(
      new ContactImpl({contact.r1Index(), contact.r2Index(), contact.r1Surface()->copy(), contact.r2Surface()->copy(),
                       contact.X_r2s_r1s(), contact.isFixed(), contact.X_b_s(), contact.ambiguityId()}));
}

Contact & Contact::operator=(const Contact & rhs)
{
  if(this == &rhs)
  {
    return *this;
  }
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

Contact::~Contact() {}

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
  return X_0_r1s(robots.robot(impl->r2Index));
}

sva::PTransformd Contact::X_0_r1s(const mc_rbdyn::Robot & robot) const
{
  return impl->X_r2s_r1s * (impl->r2Surface->X_0_s(robot));
}

sva::PTransformd Contact::X_0_r2s(const mc_rbdyn::Robots & robots) const
{
  return X_0_r2s(robots.robot(impl->r1Index));
}

sva::PTransformd Contact::X_0_r2s(const mc_rbdyn::Robot & robot) const
{
  return impl->X_r2s_r1s.inv() * (impl->r1Surface->X_0_s(robot));
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
  sva::PTransformd X_0_r1 = impl->r1Surface->X_0_s(robots.robot(impl->r1Index));
  sva::PTransformd X_0_r2 = impl->r2Surface->X_0_s(robots.robot(impl->r2Index));
  return X_0_r1 * X_0_r2.inv();
}

tasks::qp::ContactId Contact::contactId(const mc_rbdyn::Robots & /*robots*/) const
{
  return tasks::qp::ContactId(static_cast<int>(impl->r1Index), static_cast<int>(impl->r2Index),
                              impl->r1Surface->bodyName(), impl->r2Surface->bodyName());
}

mc_solver::QPContactPtr Contact::taskContact(const mc_rbdyn::Robots & robots) const
{
  const mc_rbdyn::Robot & r1 = robots.robot(impl->r1Index);
  const mc_rbdyn::Robot & r2 = robots.robot(impl->r2Index);
  unsigned int r1BodyIndex = r1.bodyIndexByName(impl->r1Surface->bodyName());
  unsigned int r2BodyIndex = r2.bodyIndexByName(impl->r2Surface->bodyName());
  sva::PTransformd X_0_b1 = r1.mbc().bodyPosW[r1BodyIndex];
  sva::PTransformd X_0_b2 = r2.mbc().bodyPosW[r2BodyIndex];
  sva::PTransformd X_b1_b2 = X_0_b2 * X_0_b1.inv();
  const auto & r1Surface = *(impl->r1Surface);
  return taskContact(robots, X_b1_b2, r1Surface.points());
}

mc_solver::QPContactPtrWPoints Contact::taskContactWPoints(const mc_rbdyn::Robots & robots,
                                                           const sva::PTransformd * X_es_rs) const
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

mc_solver::QPContactPtr Contact::taskContact(const mc_rbdyn::Robots & /*robots*/,
                                             const sva::PTransformd & X_b1_b2,
                                             const std::vector<sva::PTransformd> & surf_points) const
{
  mc_solver::QPContactPtr res;

  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Matrix3d> frames;
  for(const auto & p : surf_points)
  {
    points.push_back(p.translation());
    frames.push_back(p.rotation());
  }

  if(impl->r1Surface->type() == "planar")
  {
    res.unilateralContact =
        new tasks::qp::UnilateralContact(static_cast<int>(impl->r1Index), static_cast<int>(impl->r2Index),
                                         impl->r1Surface->bodyName(), impl->r2Surface->bodyName(), impl->ambiguityId,
                                         points, frames[0], X_b1_b2, nrConeGen, defaultFriction, impl->X_b_s);
  }
  else if(impl->r1Surface->type() == "gripper")
  {
    res.bilateralContact =
        new tasks::qp::BilateralContact(static_cast<int>(impl->r1Index), static_cast<int>(impl->r2Index),
                                        impl->r1Surface->bodyName(), impl->r2Surface->bodyName(), impl->ambiguityId,
                                        points, frames, X_b1_b2, nrConeGen, defaultFriction, impl->X_b_s);
  }
  else
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Robot's contact surface is neither planar nor gripper")
  }

  return res;
}

std::string Contact::toStr() const
{
  std::stringstream ss;
  ss << impl->r1Surface->toStr() << "/" << impl->r2Surface->toStr();
  return ss.str();
}

bool Contact::operator==(const Contact & rhs) const
{
  return (*(this->r1Surface()) == *(rhs.r1Surface())) && (*(this->r2Surface()) == *(rhs.r2Surface()));
}

bool Contact::operator!=(const Contact & rhs) const
{
  return !(*this == rhs);
}

} // namespace mc_rbdyn
