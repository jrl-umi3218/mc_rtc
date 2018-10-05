#pragma once

#include <mc_rbdyn/api.h>

#include <string>

namespace mc_rbdyn
{

/** This structure holds a flexible joint, if such a joint is part of a robot,
 * then the following dynamic constraint will be applied for the flexible joint
 * torques:
 *
 * \f{align}
 * \underline{\mathbf{\tau}} = \overline{\mathbf{\tau}} = -K\mathbf{q} -C\dot{\mathbf{q}} - O
 * \f}
 *
 */

struct MC_RBDYN_DLLAPI Flexibility
{
public:
  /** Name of the joint */
  std::string jointName;
  /** Stiffness */
  double K;
  /** Damping */
  double C;
  /** Bias */
  double O;
};

} // namespace mc_rbdyn
