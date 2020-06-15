#pragma once
#include <mc_planning/LinearControl3.h>

namespace mc_planning
{
namespace linear_control_system
{

class MC_PLANNING_DLLAPI LIPMControlByPoleAssign : public LinearControl3
{
public:
  void setStateVariables(double x_, double v_, double p_)
  {
    x << x_, v_, p_;
  }
  void getStateVariables(double & x_, double & v_, double & p_)
  {
    x_ = x(0);
    v_ = x(1);
    p_ = x(2);
  }
  void getStateVariables(double & x_, double & v_, double & p_, double & pdot)
  {
    x_ = x(0);
    v_ = x(1);
    p_ = x(2);
    pdot = u;
  }
  void setSystemMatrices(const double alpha, const double beta, const double gamma, const double cog_height);
};

} // namespace linear_control_system
} // namespace mc_planning
