/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/DistanceLimit.h>

#include <iostream>

namespace mc_rbdyn
{

bool DistanceLimit::operator==(const DistanceLimit & rhs) const
{
  return body1 == rhs.body1 && body2 == rhs.body2 && iDist == rhs.iDist && sDist == rhs.sDist;
}

bool DistanceLimit::operator!=(const DistanceLimit & rhs) const
{
  return !(*this == rhs);
}

std::ostream & operator<<(std::ostream & os, const DistanceLimit & dl)
{
  os << "DistanceLimit: " << dl.body1 << "/" << dl.body2 << " { " << dl.iDist << ", " << dl.sDist << ", " << dl.damping
     << "}";
  return os;
}

} // namespace mc_rbdyn
