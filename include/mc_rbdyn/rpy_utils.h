#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rbdyn
{

inline Eigen::Matrix3d rpyToMat(const double & r, const double & p, const double & y)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
  return sva::RotX(r)*sva::RotY(p)*sva::RotZ(y);
#pragma GCC diagnostic pop
}

inline Eigen::Matrix3d rpyToMat(const Eigen::Vector3d & rpy)
{
  return rpyToMat(rpy(0), rpy(1), rpy(2));
}

inline sva::PTransformd rpyToPT(const Eigen::Vector3d & rpy)
{
  return sva::PTransformd(rpyToMat(rpy(0), rpy(1), rpy(2)));
}

inline sva::PTransformd rpyToPT(const double & r, const double & p, const double & y)
{
  return sva::PTransformd(rpyToMat(r, p, y));
}

}
