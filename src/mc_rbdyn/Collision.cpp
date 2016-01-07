#include <mc_rbdyn/Collision.h>

#include <iostream>

namespace mc_rbdyn
{

bool operator==(const Collision & lhs, const Collision & rhs)
{
  return lhs.body1 == rhs.body1 && lhs.body2 == rhs.body2;
}

bool operator!=(const Collision & lhs, const Collision & rhs)
{
  return !(lhs == rhs);
}

std::ostream & operator<<(std::ostream & os, const Collision & col)
{
  os << "Collision: " << col.body1 << "/" << col.body2 << " { " << col.iDist << ", " << col.sDist << ", " << col.damping << "}" << std::endl;
  return os;
}

} // namespace mc_rbdyn
