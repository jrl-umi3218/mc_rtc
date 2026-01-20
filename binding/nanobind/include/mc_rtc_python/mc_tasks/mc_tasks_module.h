#pragma once
#include <nanobind/nanobind.h>

namespace mc_rtc_python
{
void bind_mc_tasks_module(nanobind::module_ & m);

void bind_MetaTask(nanobind::module_ & m);
void bind_TrajectoryTaskGeneric(nanobind::module_ & m);
void bind_PostureTask(nanobind::module_ & m);
void bind_TransformTask(nanobind::module_ & m);
void bind_GazeTask(nanobind::module_ & m);
void bind_EndEffectorTask(nanobind::module_ & m);
void bind_ImpedanceTask(nanobind::module_ & m);
void bind_PositionTask(nanobind::module_ & m);
void bind_OrientationTask(nanobind::module_ & m);
void bind_CoMTask(nanobind::module_ & m);
void bind_CoPTask(nanobind::module_ & m);
void bind_ComplianceTask(nanobind::module_ & m);
void bind_BSplineTrajectoryTask(nanobind::module_ & m); 
} // namespace mc_rtc_python
