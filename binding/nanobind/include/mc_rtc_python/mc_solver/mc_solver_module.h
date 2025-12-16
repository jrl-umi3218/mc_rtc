#pragma once
#include <nanobind/nanobind.h>

namespace mc_rtc_python
{
void bind_mc_solver_module(nanobind::module_ & m);

void bind_QPSolver(nanobind::module_ & m);
void bind_KinematicsConstraint(nanobind::module_ & m);
void bind_DynamicsConstraint(nanobind::module_ & m);
void bind_ContactConstraint(nanobind::module_ & m);
void bind_CompoundJointConstraint(nanobind::module_ & m);
void bind_CoMIncPlaneConstr(nanobind::module_ & m);
void bind_CollisionsConstraint(nanobind::module_ & m);
void bind_BoundedSpeedConstr(nanobind::module_ & m);
} // namespace mc_rtc_python

