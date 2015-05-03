#include <mc_tasks/MetaTask.h>

namespace mc_tasks
{

double extraStiffness(double error, double extraStiffness)
{
  return exp(-error)*extraStiffness;
}

}
