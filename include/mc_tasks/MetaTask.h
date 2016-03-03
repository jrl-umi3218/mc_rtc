#ifndef _H_METATASK_H_
#define _H_METATASK_H_

#include <cmath>
#include <mc_solver/qpsolver.h>

#include <mc_tasks/api.h>

namespace mc_tasks
{

MC_TASKS_DLLAPI double extraStiffness(double error, double extraStiffness);

struct MC_TASKS_DLLAPI MetaTask
{
public:
  virtual void addToSolver(mc_solver::QPSolver & solver) = 0;

  virtual void removeFromSolver(mc_solver::QPSolver & solver) = 0;

  virtual void update() = 0;
};

}

#endif
