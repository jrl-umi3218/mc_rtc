#include <iostream>
#include <mc_rtc_python/mc_tasks/mc_tasks_module.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_mc_tasks_module(nb::module_ & m)
{
  
  m.doc() = "mc_tasks bindings";
  bind_MetaTask(m);
  bind_TrajectoryTaskGeneric(m);
  bind_PostureTask(m);
  bind_TransformTask(m);
  bind_GazeTask(m);
  bind_PositionTask(m);
  bind_OrientationTask(m);
  bind_EndEffectorTask(m);
  bind_ImpedanceTask(m);
  bind_CoMTask(m);
  bind_CoPTask(m);
  bind_ComplianceTask(m);
  bind_BSplineTrajectoryTask(m);
}
} // namespace mc_rtc_python
