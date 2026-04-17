/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/ContactConstraint.h>

#include <Eigen/Core>
#include <memory>
#include <sstream>

typedef std::array<double, 3> array3d;

namespace mc_solver
{

struct ContactConstrCastResult
{
  tasks::qp::ContactAccConstr * acc;
  tasks::qp::ContactSpeedConstr * speed;
  tasks::qp::ContactPosConstr * pos;
};

ContactConstrCastResult get_contact_constr(ContactConstraint & cc)
{
  ContactConstrCastResult res{nullptr, nullptr, nullptr};
  if(cc.backend() != QPSolver::Backend::Tasks)
  {
    return res;
  }
  auto constr = cc.contactConstr();
  res.acc = dynamic_cast<tasks::qp::ContactAccConstr *>(constr);
  res.speed = dynamic_cast<tasks::qp::ContactSpeedConstr *>(constr);
  res.pos = dynamic_cast<tasks::qp::ContactPosConstr *>(constr);
  return res;
}

} // namespace mc_solver
