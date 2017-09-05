#ifndef _H_MCRBDYN_CONTACT_H_
#define _H_MCRBDYN_CONTACT_H_

#include <SpaceVecAlg/SpaceVecAlg>
#include <Tasks/QPContacts.h>

#include <memory>

#include <mc_rbdyn/api.h>
#include <mc_rtc/Configuration.h>

namespace mc_solver
{

  struct MC_RBDYN_DLLAPI QPContactPtr
  {
    QPContactPtr() : unilateralContact(nullptr), bilateralContact(nullptr) {}
    tasks::qp::UnilateralContact * unilateralContact;
    tasks::qp::BilateralContact * bilateralContact;
  };

  struct MC_RBDYN_DLLAPI QPContactPtrWPoints
  {
    QPContactPtr qpcontact_ptr;
    std::vector<sva::PTransformd> points;
  };

}

namespace mc_rbdyn
{

struct Robot;
struct Robots;
struct Surface;

MC_RBDYN_DLLAPI std::vector<sva::PTransformd> computePoints(const mc_rbdyn::Surface & robotSurface, const mc_rbdyn::Surface & envSurface, const sva::PTransformd & X_es_rs);

struct ContactImpl;

struct MC_RBDYN_DLLAPI Contact
{
public:
  constexpr static int nrConeGen = 4;
  constexpr static double defaultFriction = 0.7;
  constexpr static unsigned int nrBilatPoints = 4;
public:
  Contact(const mc_rbdyn::Robots & robots, const std::string & robotSurface, const std::string & envSurface);
  Contact(const mc_rbdyn::Robots & robots, const std::string & robotSurface, const std::string & envSurface, const sva::PTransformd & X_es_rs);
  Contact(const mc_rbdyn::Robots & robots, unsigned int r1Index, unsigned int r2Index,
            const std::string & r1Surface, const std::string & r2Surface,
            const sva::PTransformd * X_r2s_r1s = nullptr,
            const sva::PTransformd & Xbs = sva::PTransformd::Identity(), int ambiguityId = -1);
private:
  Contact(const mc_rbdyn::Robots & robots, const std::string & robotSurface, const std::string & envSurface, const sva::PTransformd & X_es_rs, bool is_fixed);
public:
  Contact(const Contact & contact);
  Contact & operator=(const Contact &);
  ~Contact();

  unsigned int r1Index() const;

  unsigned int r2Index() const;

  const std::shared_ptr<mc_rbdyn::Surface> & r1Surface() const;

  const std::shared_ptr<mc_rbdyn::Surface> & r2Surface() const;

  const sva::PTransformd & X_r2s_r1s() const;

  void X_r2s_r1s(const sva::PTransformd & in);

  const sva::PTransformd & X_b_s() const;

  const int & ambiguityId() const;

  bool isFixed() const;

  std::pair<std::string, std::string> surfaces() const;

  sva::PTransformd X_0_r1s(const mc_rbdyn::Robot & robot) const;
  sva::PTransformd X_0_r1s(const mc_rbdyn::Robots & robots) const;

  sva::PTransformd X_0_r2s(const mc_rbdyn::Robot & robot) const;
  sva::PTransformd X_0_r2s(const mc_rbdyn::Robots & robots) const;

  std::vector<sva::PTransformd> r1Points();

  std::vector<sva::PTransformd> r2Points();

  sva::PTransformd compute_X_r2s_r1s(const mc_rbdyn::Robots & robots) const;

  tasks::qp::ContactId contactId(const mc_rbdyn::Robots & robots) const;

  mc_solver::QPContactPtr taskContact(const mc_rbdyn::Robots & robots) const;

  mc_solver::QPContactPtrWPoints taskContactWPoints(const mc_rbdyn::Robots & robots, const sva::PTransformd * X_es_rs = nullptr) const;

  std::string toStr() const;

  bool operator==(const Contact & rhs) const;
  bool operator!=(const Contact & rhs) const;
private:
  std::unique_ptr<ContactImpl> impl;
  mc_solver::QPContactPtr taskContact(const mc_rbdyn::Robots & robots, const sva::PTransformd & X_b1_b2, const std::vector<sva::PTransformd> & points) const;
public:
  static mc_rbdyn::Contact load(const mc_rbdyn::Robots & robots, const mc_rtc::Configuration & config);

  static std::vector<mc_rbdyn::Contact> loadVector(const mc_rbdyn::Robots & robots, const mc_rtc::Configuration & config);
};

}

#endif
