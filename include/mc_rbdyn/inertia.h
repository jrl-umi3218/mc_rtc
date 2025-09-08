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

/**
 * @brief Compute the inertia of a superellipsoid by interpolating between box and ellipsoid inertia.
 *
 * The superellipsoid is defined by its bounding box (size) and exponents epsilon1 and epsilon2.
 * This function approximates the inertia tensor by interpolating between the inertia of a box
 * (epsilon1, epsilon2 → 0) and an ellipsoid (epsilon1, epsilon2 = 1) with the same bounding box.
 *
 * @param mass The mass of the superellipsoid.
 * @param size The size (bounding box) as an Eigen::Vector3d (x, y, z).
 * @param epsilon1 The first superellipsoid exponent (controls roundness).
 * @param epsilon2 The second superellipsoid exponent (controls roundness).
 * @return sva::RBInertiad The computed inertia.
 */
sva::RBInertiad computeSuperEllipsoidInertia(double mass,
                                             const Eigen::Vector3d & size,
                                             double epsilon1,
                                             double epsilon2);

} // namespace mc_rbdyn
