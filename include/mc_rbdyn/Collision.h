/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>

#include <optional>
#include <string>
#include <vector>

/** \class Collision
 * \brief Used to define a collision constraint between two bodies
 */

namespace mc_rbdyn
{

struct MC_RBDYN_DLLAPI Collision
{
  Collision() : body1("NONE"), body2("NONE") {}
  Collision(const std::string & b1,
            const std::string & b2,
            double i,
            double s,
            double d,
            const std::optional<std::vector<std::string>> & r1Joints = {},
            const std::optional<std::vector<std::string>> & r2Joints = {},
            bool r1JointsInactive = false,
            bool r2JointsInactive = false)
  : body1(b1), body2(b2), iDist(i), sDist(s), damping(d), r1Joints(r1Joints), r2Joints(r2Joints),
    r1JointsInactive(r1JointsInactive), r2JointsInactive(r2JointsInactive)
  {
  }
  std::string body1; /** First body in the constraint */
  std::string body2; /** Second body in the constraint */
  double iDist; /** Interaction distance */
  double sDist; /** Security distance */
  double damping; /** Damping (0 is automatic) */
  /**
   * Active/Inactive joints in the first robot:
   * - no value specified = all joints selected
   * - value specified:
   *   - if r1JointsInactive = false : specified joints are treated as active
   *   - if r1JointsInactive = true : specified joints are treated as inactive
   */
  std::optional<std::vector<std::string>> r1Joints;
  std::optional<std::vector<std::string>>
      r2Joints; /** Active/Inactive joints in the second robot, ignored if r1 == r2 */
  bool r1JointsInactive = false; /** When true the selected joints in r1ActiveJoints are considered inactive */
  bool r2JointsInactive = false; /** When true the selected joints in r2ActiveJoints are considered inactive */
  inline bool isNone() { return body1 == "NONE" && body2 == "NONE"; }

  bool operator==(const Collision & rhs) const;
  bool operator!=(const Collision & rhs) const;
};

MC_RBDYN_DLLAPI std::ostream & operator<<(std::ostream & os, const Collision & c);

} // namespace mc_rbdyn
