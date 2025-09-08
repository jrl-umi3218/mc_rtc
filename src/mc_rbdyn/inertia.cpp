#include <mc_rbdyn/inertia.h>

namespace mc_rbdyn
{

sva::RBInertiad computeBoxInertia(double mass, const Eigen::Vector3d & size)
{
  /* Compute principal moments of inertia for a box:
     I_xx = (1/12) * mass * (size.y^2 + size.z^2)
     I_yy = (1/12) * mass * (size.x^2 + size.z^2)
     I_zz = (1/12) * mass * (size.x^2 + size.y^2)
     Construct the inertia matrix as a diagonal matrix */
  double I_xx = (1.0 / 12.0) * mass * (size.y() * size.y() + size.z() * size.z());
  double I_yy = (1.0 / 12.0) * mass * (size.x() * size.x() + size.z() * size.z());
  double I_zz = (1.0 / 12.0) * mass * (size.x() * size.x() + size.y() * size.y());
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia(0, 0) = I_xx;
  inertia(1, 1) = I_yy;
  inertia(2, 2) = I_zz;
  return sva::RBInertiad(mass, Eigen::Vector3d::Zero(), inertia);
}

sva::RBInertiad computeSphereInertia(double mass, double radius)
{
  return sva::RBInertiad(mass, Eigen::Vector3d::Zero(),
                         (2. / 5.) * mass * radius * radius * Eigen::Matrix3d::Identity());
}

sva::RBInertiad computeCylinderInertia(double mass, double radius, double length)
{
  double I_xx = (1.0 / 12.0) * mass * (3 * radius * radius + length * length);
  double I_yy = I_xx;
  double I_zz = 0.5 * mass * radius * radius;
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia(0, 0) = I_xx;
  inertia(1, 1) = I_yy;
  inertia(2, 2) = I_zz;
  return sva::RBInertiad(mass, Eigen::Vector3d::Zero(), inertia);
}

sva::RBInertiad computeSuperEllipsoidInertia(double mass,
                                             const Eigen::Vector3d & size,
                                             double epsilon1,
                                             double epsilon2)
{
  // Box inertia
  double I_xx_box = (1.0 / 12.0) * mass * (size.y() * size.y() + size.z() * size.z());
  double I_yy_box = (1.0 / 12.0) * mass * (size.x() * size.x() + size.z() * size.z());
  double I_zz_box = (1.0 / 12.0) * mass * (size.x() * size.x() + size.y() * size.y());

  // Ellipsoid inertia (semi-axes a, b, c = size/2)
  double a = size.x() / 2.0, b = size.y() / 2.0, c = size.z() / 2.0;
  double I_xx_ell = (1.0 / 5.0) * mass * (b * b + c * c);
  double I_yy_ell = (1.0 / 5.0) * mass * (a * a + c * c);
  double I_zz_ell = (1.0 / 5.0) * mass * (a * a + b * b);

  // Interpolation factor: 0 = box, 1 = ellipsoid
  double alpha = std::clamp((epsilon1 + epsilon2) / 2.0, 0.0, 1.0);

  double I_xx = (1 - alpha) * I_xx_box + alpha * I_xx_ell;
  double I_yy = (1 - alpha) * I_yy_box + alpha * I_yy_ell;
  double I_zz = (1 - alpha) * I_zz_box + alpha * I_zz_ell;

  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia(0, 0) = I_xx;
  inertia(1, 1) = I_yy;
  inertia(2, 2) = I_zz;
  return sva::RBInertiad(mass, Eigen::Vector3d::Zero(), inertia);
}

} // namespace mc_rbdyn
