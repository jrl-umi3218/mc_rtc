#pragma once
#include <mc_rbdyn/api.h>
#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rbdyn
{

/*
 * Computes the inertia of a box-shaped rigid body.
 *
 * Assumptions:
 * - The box is a solid, homogeneous object.
 * - The mass is uniformly distributed.
 * - The inertia is computed about the center of mass.
 * - The box is aligned with the coordinate axes.
 *
 * @param mass Mass of the box.
 * @param size Dimensions of the box (length, width, height).
 * @return Inertia of the box as sva::RBInertiad.
 */
MC_RBDYN_DLLAPI sva::RBInertiad computeBoxInertia(double mass, const Eigen::Vector3d & size);

/*
 * Computes the inertia of a sphere-shaped rigid body.
 *
 * Assumptions:
 * - The sphere is solid and homogeneous.
 * - The mass is uniformly distributed.
 * - The inertia is computed about the center of mass.
 *
 * @param mass Mass of the sphere.
 * @param radius Radius of the sphere.
 * @return Inertia of the sphere as sva::RBInertiad.
 */
sva::RBInertiad computeSphereInertia(double mass, double radius);

/**
 * @brief Computes the inertia of a solid cylinder.
 *
 * This function calculates the rotational inertia tensor for a cylinder given its mass, radius, and length.
 *
 * @param mass Mass of the cylinder.
 * @param radius Radius of the cylinder.
 * @param length Length of the cylinder.
 * @return sva::RBInertiad Inertia of the cylinder.
 */
sva::RBInertiad computeCylinderInertia(double mass, double radius, double length);

} // namespace mc_rbdyn
