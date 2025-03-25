/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>
#include <mc_tvm/fwd.h>

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/fwd.h>

#include <Eigen/Core>

#include <tvm/graph/abstract/Node.h>

namespace mc_tvm
{

/** A Feasible Polytope is a set of planes that can be used as a constraint 
 * for a variable. It is created from an mc_rbdyn::Contact.
 *
 * Outputs:
 * - Normals: Matrix of the planes normals
 * - Offsets: Vector of the planes offsets
 */
struct MC_TVM_DLLAPI FeasiblePolytope : public tvm::graph::abstract::Node<FeasiblePolytope>
{

  // XXX for now not using this, idk what the purpose is
  // maybe for uniqueness?
private:
  struct NewPolytopeToken
  {
  };

public:
  SET_OUTPUTS(FeasiblePolytope, Polytope)
  SET_UPDATES(FeasiblePolytope, Polytope)

  /**
   * @brief Construct a new Feasible Polytope object
   * 
   * @param contact rbdyn contact associated
   */
  FeasiblePolytope(/*NewPolytopeToken,*/
                   const mc_rbdyn::Contact & contact);

  FeasiblePolytope(const FeasiblePolytope &) = delete;
  FeasiblePolytope & operator=(const FeasiblePolytope &) = delete;

  
  /** Access the normals matrix of the polytope */
  inline const Eigen::MatrixX3d & normals() const noexcept { return normals_; }

  /** Access the offsets vector of the polytope */
  inline const Eigen::VectorXd & offsets() const noexcept { return offsets_; }

  /** Access the associated contact */
  inline const mc_rbdyn::Contact & contact() const noexcept { return contact_; }

  void updatePolytope();

private:
  /** Parent instance */
  const mc_rbdyn::Contact & contact_;
  
  /** Set of planes */
  Eigen::MatrixX3d normals_;
  Eigen::VectorXd offsets_;

  
};

} // namespace mc_tvm
