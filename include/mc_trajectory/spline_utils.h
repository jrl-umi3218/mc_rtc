#ifndef _H_SPLINEUTILS_H_
#define _H_SPLINEUTILS_H_

#include <Eigen/Core>

namespace mc_trajectory
{

Eigen::MatrixXd generateInterpolatedWaypoints(const Eigen::Vector3d & start, const Eigen::Vector3d & stop, unsigned int nrWP);

}

#endif
