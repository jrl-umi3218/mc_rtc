#ifndef _H_MCRBDYN_CONTACT_H_
#define _H_MCRBDYN_CONTACT_H_

#include <SpaceVecAlg/SpaceVecAlg>
#include <Tasks/QPContacts.h>

#include <mc_rbdyn/surface.h>
#include <mc_rbdyn/contact_transform.h>

namespace mc_rbdyn
{

struct Robot;
struct Robots;

std::vector<sva::PTransformd> computePoints(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface, const sva::PTransformd & X_es_rs);

std::vector<double> jointParam(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface, const sva::PTransformd & X_es_rs);

struct Contact
{
public:
  Contact(const std::shared_ptr<mc_rbdyn::Surface> & robotSurface, const std::shared_ptr<mc_rbdyn::Surface> & envSurface);
  Contact(const std::shared_ptr<mc_rbdyn::Surface> & robotSurface, const std::shared_ptr<mc_rbdyn::Surface> & envSurface, const sva::PTransformd & X_es_rs);
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

  sva::PTransformd compute_X_es_rs(const mc_rbdyn::Robot & robot, const mc_rbdyn::Robot & env) const;
  sva::PTransformd compute_X_es_rs(const mc_rbdyn::Robot & robot, const mc_rbdyn::Robot & env, const mc_rbdyn::Surface & robotSurfaceIn) const;

  std::vector<double> computeJointParam();
  std::vector<double> computeJointParam(const mc_rbdyn::Surface & robotSurfaceIn);

  tasks::qp::ContactId contactId(const mc_rbdyn::Robot & robot, const mc_rbdyn::Robot & env);

  std::string toStr() const;
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

struct MRContact
{
public:
  MRContact(unsigned int r1Index, unsigned int r2Index,
            const std::shared_ptr<mc_rbdyn::Surface> & r1Surface,
            const std::shared_ptr<mc_rbdyn::Surface> & r2Surface,
            const sva::PTransformd * X_r2s_r1s = 0,
            const sva::PTransformd & Xbs = sva::PTransformd::Identity(), int ambiguityId = -1);

  bool isFixed() const;

  std::pair<std::string, std::string> surfaceNames() const;

  sva::PTransformd X_0_r1s(const Robots & robots) const;

  sva::PTransformd X_0_r2s(const Robots & robots) const;

  std::vector<sva::PTransformd> r1Points();

  std::vector<sva::PTransformd> r2Points();

  sva::PTransformd compute_X_r2s_r1s(const std::vector<Robot> & robots);
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

}

#endif
