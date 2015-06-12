#ifndef _H_MCRBDYN_CONTACT_H_
#define _H_MCRBDYN_CONTACT_H_

#include <SpaceVecAlg/SpaceVecAlg>
#include <Tasks/QPContacts.h>

#include <memory>

namespace mc_rbdyn
{

struct Robot;
struct Robots;
struct Surface;

std::vector<sva::PTransformd> computePoints(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface, const sva::PTransformd & X_es_rs);

struct Contact
{
public:
  Contact(const std::shared_ptr<mc_rbdyn::Surface> & robotSurface, const std::shared_ptr<mc_rbdyn::Surface> & envSurface);
  Contact(const std::shared_ptr<mc_rbdyn::Surface> & robotSurface, const std::shared_ptr<mc_rbdyn::Surface> & envSurface, const sva::PTransformd & X_es_rs);
  Contact(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface);
  Contact(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface, const sva::PTransformd & X_es_rs);
  Contact(unsigned int r1Index, unsigned int r2Index,
            const std::shared_ptr<mc_rbdyn::Surface> & r1Surface,
            const std::shared_ptr<mc_rbdyn::Surface> & r2Surface,
            const sva::PTransformd * X_r2s_r1s = 0,
            const sva::PTransformd & Xbs = sva::PTransformd::Identity(), int ambiguityId = -1);
private:
  Contact(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface, const sva::PTransformd & X_es_rs, bool is_fixed);
public:
  Contact(const Contact & contact);
  Contact & operator=(const Contact &);

  bool isFixed();

  std::pair<std::string, std::string> surfaces() const;

  sva::PTransformd X_0_r1s(const mc_rbdyn::Robots & robots) const;

  sva::PTransformd X_0_r2s(const mc_rbdyn::Robots & robots) const;

  std::vector<sva::PTransformd> r1Points();

  std::vector<sva::PTransformd> r2Points();

  sva::PTransformd compute_X_r2s_r1s(const mc_rbdyn::Robots & robots) const;

  tasks::qp::ContactId contactId(const mc_rbdyn::Robots & robots) const;

  std::string toStr() const;
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

bool operator==(const Contact & lhs, const Contact & rhs);

bool operator!=(const Contact & lhs, const Contact & rhs);

}

#endif
