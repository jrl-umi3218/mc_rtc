#pragma once

#include <Eigen/Core>
#include <mc_solver/ContactConstraint.h>

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
  ContactConstrCastResult res {nullptr, nullptr, nullptr};
  res.acc = dynamic_cast<tasks::qp::ContactAccConstr*>(cc.contactConstr.get());
  res.speed = dynamic_cast<tasks::qp::ContactSpeedConstr*>(cc.contactConstr.get());
  res.pos = dynamic_cast<tasks::qp::ContactPosConstr*>(cc.contactConstr.get());
  return res;
}

}
