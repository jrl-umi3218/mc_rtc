/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/MaxDist.h>

#include <iostream>

namespace mc_rbdyn
{

bool MaxDist::operator==(const MaxDist & rhs) const
{
  return body1 == rhs.body1 && body2 == rhs.body2;
}

bool MaxDist::operator!=(const MaxDist & rhs) const
{
  return !(*this == rhs);
}

std::ostream & operator<<(std::ostream & os, const MaxDist & m)
{
  os << "MaxDist: " << m.body1 << "/" << m.body2 << " { " << m.iDist << ", " << m.sDist << ", " << m.damping << "}";
  return os;
}

} // namespace mc_rbdyn
