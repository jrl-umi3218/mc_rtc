#pragma once

#include <Eigen/Core>
#include <mc_rbdyn/Collision.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/stance.h>
#include <mc_rbdyn/StanceConfig.h>

#include <memory>
#include <sstream>

namespace
{

template<typename T>
struct NoOpDeleter
{
  void operator()(T *) const {}
};

}

namespace mc_rbdyn
{

std::string CollisionToString(const Collision& c)
{
  std::stringstream ss;
  ss << c;
  return ss.str();
}

typedef std::shared_ptr<mc_rbdyn::RobotModule> RobotModulePtr;

template<typename ... Args>
RobotModulePtr get_robot_module(const std::string & name, const Args & ... args)
{
  return RobotLoader::get_robot_module(name, args...);
}

Robots& const_cast_robots(const Robots& rhs)
{
  return const_cast<Robots&>(rhs);
}

Robot& const_cast_robot(const Robot& rhs)
{
  return const_cast<Robot&>(rhs);
}

ForceSensor& const_cast_force_sensor(const ForceSensor & fs)
{
  return const_cast<ForceSensor&>(fs);
}

Surface& const_cast_surface(const Surface& rhs)
{
  return const_cast<Surface&>(rhs);
}

Contact& const_cast_contact(const Contact& rhs)
{
  return const_cast<Contact&>(rhs);
}

std::vector<Contact>& const_cast_contact_vector(const std::vector<Contact>& rhs)
{
  return const_cast<std::vector<Contact>&>(rhs);
}

void contact_vector_set_item(std::vector<Contact> & v, unsigned int idx, const Contact & c)
{
  v[idx] = c;
}

Stance& const_cast_stance(const Stance& rhs)
{
  return const_cast<Stance&>(rhs);
}

IdentityContactAction * dynamic_cast_ica(StanceAction * p)
{
  return dynamic_cast<IdentityContactAction*>(p);
}

AddContactAction * dynamic_cast_aca(StanceAction * p)
{
  return dynamic_cast<AddContactAction*>(p);
}

RemoveContactAction * dynamic_cast_rca(StanceAction * p)
{
  return dynamic_cast<RemoveContactAction*>(p);
}

std::shared_ptr<StanceAction> sa_fake_shared(StanceAction * p)
{
  return std::shared_ptr<StanceAction>(p, NoOpDeleter<StanceAction>());
}

typedef Stance* StanceRawPtr;

void scbc_vector_set_item(std::vector<StanceConfig::BodiesCollisionConf>&v, unsigned int idx, StanceConfig::BodiesCollisionConf&c)
{
  v[idx] = c;
}

typedef Eigen::Vector3d (*pos_callback_t)(const sva::PTransformd&, const sva::PTransformd&, const Eigen::Vector3d&);
typedef Eigen::Vector3d (*user_pos_callback_t)(const sva::PTransformd&, const sva::PTransformd&, const Eigen::Vector3d&, void*);

Eigen::Vector3d call_pos(StanceConfig::WaypointConf & wpc, const sva::PTransformd&start, const sva::PTransformd&end, const Eigen::Vector3d&N)
{
  return wpc.pos(start, end, N);
}

void set_pos(StanceConfig::WaypointConf & wpc, user_pos_callback_t fn, void * data)
{
  wpc.pos = std::bind(fn, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, data);
}

std::shared_ptr<Robots> robots_fake_shared(Robots * p)
{
  return std::shared_ptr<Robots>(p, NoOpDeleter<Robots>());
}

PolygonInterpolator * polygonInterpolatorFromTuplePairs(const std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> & pairs)
{
  std::vector<PolygonInterpolator::tuple_pair_t> tuple_pairs;
  for(const auto & p : pairs)
  {
    tuple_pairs.push_back({{{p.first.first, p.first.second}}, {{p.second.first, p.second.second}}});
  }
  return new PolygonInterpolator(tuple_pairs);
}

std::vector<double> robotModuleDefaultAttitude(RobotModulePtr rm)
{
  auto attitude = rm->default_attitude();
  std::vector<double> ret(7);
  for(size_t i = 0; i < 7; ++i)
  {
    ret[i] = attitude[i];
  }
  return ret;
}

}
