/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifndef _H_MCRBDYN_CONTACT_H_
#define _H_MCRBDYN_CONTACT_H_

#include <mc_rbdyn/api.h>
#include <mc_rtc/Configuration.h>

#include <mc_tvm/fwd.h>

#include <Tasks/QPContacts.h>

#include <SpaceVecAlg/SpaceVecAlg>
#include <optional>

#include <memory>

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

} // namespace mc_solver

namespace mc_rbdyn
{

struct Robot;
struct Robots;
struct Surface;

MC_RBDYN_DLLAPI std::vector<sva::PTransformd> computePoints(const mc_rbdyn::Surface & robotSurface,
                                                            const mc_rbdyn::Surface & envSurface,
                                                            const sva::PTransformd & X_es_rs);

struct FeasiblePolytope
{
  Eigen::MatrixXd planeNormals;
  Eigen::VectorXd planeConstants;
};

struct ContactImpl;

struct MC_RBDYN_DLLAPI Contact
{
public:
  constexpr static int nrConeGen = 4; // FIXME: use it in tvmqpsolver...
  constexpr static double defaultFriction = 0.7;
  constexpr static unsigned int nrBilatPoints = 4;

public:
  Contact(const mc_rbdyn::Robots & robots,
          const std::string & robotSurface,
          const std::string & envSurface,
          double friction = defaultFriction);
  Contact(const mc_rbdyn::Robots & robots,
          const std::string & robotSurface,
          const std::string & envSurface,
          const sva::PTransformd & X_es_rs,
          double friction = defaultFriction);
  Contact(const mc_rbdyn::Robots & robots,
          unsigned int r1Index,
          unsigned int r2Index,
          const std::string & r1Surface,
          const std::string & r2Surface,
          double friction = defaultFriction,
          int ambiguityId = -1);
  Contact(const mc_rbdyn::Robots & robots,
          unsigned int r1Index,
          unsigned int r2Index,
          const std::string & r1Surface,
          const std::string & r2Surface,
          const sva::PTransformd & X_r2s_r1s,
          double friction = defaultFriction,
          int ambiguityId = -1);
  Contact(const mc_rbdyn::Robots & robots,
          unsigned int r1Index,
          unsigned int r2Index,
          const std::string & r1Surface,
          const std::string & r2Surface,
          const sva::PTransformd & X_r2s_r1s,
          const sva::PTransformd & X_b_s,
          double friction = defaultFriction,
          int ambiguityId = -1);

private:
  Contact(const mc_rbdyn::Robots & robots,
          const std::string & robotSurface,
          const std::string & envSurface,
          const sva::PTransformd & X_es_rs,
          double friction,
          bool is_fixed);

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

  /** Return the contact friction */
  double friction() const;

  /** Set the contact friction */
  void friction(double friction);

  void feasiblePolytopeR1(const FeasiblePolytope & polytope);
  const std::optional<FeasiblePolytope> & feasiblePolytopeR1() const noexcept;

  void feasiblePolytopeR2(const FeasiblePolytope & polytope);
  const std::optional<FeasiblePolytope> & feasiblePolytopeR2() const noexcept;

  /** Get the TVM polytope associated to robot 1 of this contact
   *
   * FIXME Returns a non-const reference from a const method because it is most often used to register dependencies
   * between TVM nodes which require non-const objects
   */
  mc_tvm::FeasiblePolytope & tvmPolytopeR1() const;

  /** Get the TVM polytope associated to robot 2 of this contact
   */
  mc_tvm::FeasiblePolytope & tvmPolytopeR2() const;

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

  mc_solver::QPContactPtrWPoints taskContactWPoints(const mc_rbdyn::Robots & robots,
                                                    const sva::PTransformd * X_es_rs = nullptr) const;

  std::string toStr() const;

  /** If this is a contact r1::s1/r2::s2, this returns r2::s2/r1::s1 */
  Contact swap(const mc_rbdyn::Robots & robots) const;

  inline const Eigen::Vector6d & dof() const noexcept { return dof_; }

  inline void dof(const Eigen::Vector6d & dof) noexcept { dof_ = dof; }

  bool operator==(const Contact & rhs) const;
  bool operator!=(const Contact & rhs) const;

private:
  std::unique_ptr<ContactImpl> impl;
  Eigen::Vector6d dof_ = Eigen::Vector6d::Ones();
  mc_solver::QPContactPtr taskContact(const mc_rbdyn::Robots & robots,
                                      const sva::PTransformd & X_b1_b2,
                                      const std::vector<sva::PTransformd> & points) const;

  //! If present superseeds friction cone constraints
  // feasible polytope for r1 of contact
  std::optional<FeasiblePolytope> feasiblePolytopeR1_;

  // feasible polytope for r2 of contact
  std::optional<FeasiblePolytope> feasiblePolytopeR2_;

  // Secures threaded access to the contact object
  mutable std::mutex contactMutex_;

  /* mutable to allow initialization in const method */
  mutable mc_tvm::PolytopePtr tvm_polytopeR1_;
  mutable mc_tvm::PolytopePtr tvm_polytopeR2_;

public:
  static mc_rbdyn::Contact load(const mc_rbdyn::Robots & robots, const mc_rtc::Configuration & config);

  static std::vector<mc_rbdyn::Contact> loadVector(const mc_rbdyn::Robots & robots,
                                                   const mc_rtc::Configuration & config);
};

} // namespace mc_rbdyn

#endif
