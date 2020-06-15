#include <mc_planning/LIPMControlByPoleAssign.h>

namespace mc_planning
{
namespace linear_control_system
{

void LIPMControlByPoleAssign::setSystemMatrices(const double alpha,
                                                const double beta,
                                                const double gamma,
                                                const double omega2)
{
  A << 0.0, 1.0, 0.0, omega2, 0.0, -omega2, 0.0, 0.0, 0.0;

  B << 0.0, 0.0, 1.0;

  C << 0.0, 0.0, 1.0;

  K << (alpha + beta + gamma) + alpha * beta * gamma / omega2,
      (alpha * beta + beta * gamma + gamma * alpha) / omega2 + 1.0, -(alpha + beta + gamma);
}

} // namespace linear_control_system
} // namespace mc_planning
