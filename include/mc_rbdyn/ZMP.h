/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/api.h>
#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rbdyn
{
/**
 * @brief Actual ZMP computation from net total wrench and the ZMP plane
 *
 * @param netTotalWrench Total wrench for all links in contact
 * @param plane_p Arbitrary point on the ZMP plane
 * @param plane_n Normal to the ZMP plane (normalized)
 * @param minimalNetNormalForce[N] Mininal force above which the ZMP computation
 * is considered valid. Must be >0 (prevents a divide by zero).
 *
 * @return zmp expressed in the requested plane
 *
 * @throws To prevent dividing by zero, throws if the projected force is below minimalNetNormalForce newton.
 * This is highly unlikely to happen and would likely indicate indicate that you are computing a ZMP from
 * invalid forces (such as with the robot in the air).
 *
 * \anchor zmpDoc
 */
Eigen::Vector3d MC_RBDYN_DLLAPI zmp(const sva::ForceVecd & netTotalWrench,
                                    const Eigen::Vector3d & plane_p,
                                    const Eigen::Vector3d & plane_n,
                                    double minimalNetNormalForce = 1.);

/**
 * @brief ZMP computation from net total wrench and a frame
 *
 * See \ref zmpDoc
 *
 * @param netTotalWrench
 * @param zmpFrame Frame used for ZMP computation. The convention here is
 * that the contact frame should have its z-axis pointing in the normal
 * direction of the contact towards the robot.
 *
 * @throws To prevent dividing by zero, throws if the projected force is below minimalNetNormalForce newton. This is
 * highly unlikely to happen and would likely indicate indicate that you are computing a ZMP from invalid forces (such
 * as with the robot in the air).
 *
 * @return ZMP expressed in the plane defined by the zmpFrame frame.
 */
Eigen::Vector3d MC_RBDYN_DLLAPI zmp(const sva::ForceVecd & netTotalWrench,
                                    const sva::PTransformd & zmpFrame,
                                    double minimalNetNormalForce = 1.);
} // namespace mc_rbdyn
