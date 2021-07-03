#include <mc_planning/LIPMControlByPoleAssignWithExternalForce.h>

namespace mc_planning
{
namespace linear_control_system
{

LIPMControlByPoleAssignWithExternalForce::LIPMControlByPoleAssignWithExternalForce(void) : B2(Eigen::Vector3d::Zero())
{
  LinearControl3::Initialize();
}

void LIPMControlByPoleAssignWithExternalForce::Initialize(void)
{
  B2.setZero();
  LinearControl3::Initialize();
}

void LIPMControlByPoleAssignWithExternalForce::setSystemMatrices(const double alpha,
                                                                 const double beta,
                                                                 const double gamma,
                                                                 const double omega2,
                                                                 const double total_mass)
{
  LIPMControlByPoleAssign::setSystemMatrices(alpha, beta, gamma, omega2);

  B2 << 0.0, 1.0 / total_mass, 0.0;
}

void LIPMControlByPoleAssignWithExternalForce::update(const double Fext, const double dt)
{
  y = (C * x)(0);

  double u = K.dot(x);
  Eigen::Vector3d dx(A * x + B * u + B2 * Fext);
  x += dx * dt;
}

void LIPMControlByPoleAssignWithExternalForce::update(const Eigen::Vector3d & x_ref, const double Fext, const double dt)
{
  y = (C * x)(0);

  double u = K.dot(x - x_ref);
  Eigen::Vector3d dx(A * x + B * u + B2 * Fext);
  x += dx * dt;
}

} // namespace linear_control_system
} // namespace mc_planning
