#include <iostream>
#include <mc_rtc_python/mc_solver/mc_solver_module.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_mc_solver_module(nb::module_ & m)
{
  m.doc() = "mc_solver bindings";
  bind_QPSolver(m);
  bind_KinematicsConstraint(m);
  bind_DynamicsConstraint(m);
  bind_ContactConstraint(m);
  bind_CompoundJointConstraint(m);
  bind_CoMIncPlaneConstr(m);
  bind_CollisionsConstraint(m);
  bind_BoundedSpeedConstr(m);
}

} // namespace mc_rtc_python
