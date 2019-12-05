#include <Eigen/Core>

/** Gravity and world vertical contants.
 *
 */
namespace mc_rbdyn
{

namespace world
{
constexpr double GRAVITY = 9.80665; // ISO 80000-3

const Eigen::Vector3d gravity = Eigen::Vector3d{0., 0., -GRAVITY};
const Eigen::Vector3d vertical = Eigen::Vector3d{0., 0., 1.};
} // namespace world

} // namespace mc_rbdyn
