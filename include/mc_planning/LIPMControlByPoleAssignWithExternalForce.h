#pragma once
#include "LIPMControlByPoleAssign.h"

namespace mc_planning
{
namespace linear_control_system
{

class MC_PLANNING_DLLAPI LIPMControlByPoleAssignWithExternalForce : public LIPMControlByPoleAssign
{
private:
  Eigen::Vector3d B2;

public:
  LIPMControlByPoleAssignWithExternalForce(void);

  void Initialize(void);

  void update(const double Fext, const double dt);
  void update(const Eigen::Vector3d & x_ref, const double Fext, const double dt);
  void setSystemMatrices(const double alpha,
                         const double beta,
                         const double gamma,
                         const double omega,
                         const double total_mass);
};

} // namespace linear_control_system
} // namespace mc_planning
