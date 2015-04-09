#ifndef _H_MCRBDYN_CONTACT_H_
#define _H_MCRBDYN_CONTACT_H_

#include <SpaceVecAlg/SpaceVecAlg>
#include <Tasks/QPContacts.h>

#include <mc_rbdyn/surface.h>
#include <mc_rbdyn/contact_transform.h>

namespace mc_rbdyn
{

struct Robot;

std::vector<sva::PTransformd> computePoints(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface, const sva::PTransformd & X_es_rs);

std::vector<double> jointParam(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface, const sva::PTransformd & X_es_rs);

struct Contact
{
public:
  Contact(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface);
  Contact(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface, const sva::PTransformd & X_es_rs);
private:
  Contact(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface, const sva::PTransformd & X_es_rs, bool is_fixed);
public:
  Contact(const Contact & contact);
  Contact & operator=(const Contact &);

  bool isFixed();

  std::pair<std::string, std::string> surfaces();

  sva::PTransformd X_0_rs(const mc_rbdyn::Robot & env);

  std::vector<sva::PTransformd> points();
  std::vector<sva::PTransformd> points(const mc_rbdyn::Surface & robotSurfaceIn);

  sva::PTransformd compute_X_es_rs(const mc_rbdyn::Robot & robot, const mc_rbdyn::Robot & env);
  sva::PTransformd compute_X_es_rs(const mc_rbdyn::Robot & robot, const mc_rbdyn::Robot & env, const mc_rbdyn::Surface & robotSurfaceIn);

  std::vector<double> computeJointParam();
  std::vector<double> computeJointParam(const mc_rbdyn::Surface & robotSurfaceIn);

  tasks::qp::ContactId contactId(const mc_rbdyn::Robot & robot, const mc_rbdyn::Robot & env);

  std::string toStr();
public:
  std::shared_ptr<mc_rbdyn::Surface> robotSurface;
  std::shared_ptr<mc_rbdyn::Surface> envSurface;
  sva::PTransformd X_es_rs;
  bool is_fixed;
};

inline bool operator==(const Contact & lhs, const Contact & rhs)
{
  return *(lhs.robotSurface) == *(rhs.robotSurface) && *(lhs.envSurface) == *(lhs.envSurface);
}

inline bool operator!=(const Contact & lhs, const Contact & rhs)
{
  return not(lhs == rhs);
}

/*TODO Port MRContact*/

}

#endif
