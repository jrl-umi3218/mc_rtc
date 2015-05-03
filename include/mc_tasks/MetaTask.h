#ifndef _H_METATASK_H_
#define _H_METATASK_H_

#include <cmath>
#include <Tasks/QPSolver.h>

namespace mc_tasks
{

double extraStiffness(double error, double extraStiffness);

struct MetaTask
{
public:
  virtual void addToSolver(tasks::qp::QPSolver & solver) = 0;

  virtual void removeFromSolver(tasks::qp::QPSolver & solver) = 0;

  virtual void update() = 0;
};

}

#endif
