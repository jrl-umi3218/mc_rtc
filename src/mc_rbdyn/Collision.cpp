#include <mc_rbdyn/Collision.h>

#include <iostream>

namespace mc_rbdyn
{

bool Collision::operator==(const Collision & rhs) const
{
  return body1 == rhs.body1 && body2 == rhs.body2;
}

bool Collision::operator!=(const Collision & rhs) const
{
  return !(*this == rhs);
}

std::ostream & operator<<(std::ostream & os, const Collision & col)
{
  os << "Collision: " << col.body1 << "/" << col.body2 << " { " << col.iDist << ", " << col.sDist << ", " << col.damping
     << "}";
  return os;
}

} // namespace mc_rbdyn
