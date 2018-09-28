#include <mc_rbdyn/SCHAddon.h>

namespace sch
{

namespace mc_rbdyn
{

void transform(S_Object & obj, const sva::PTransformd & t)
{
  sch::Matrix4x4 m;
  const Eigen::Matrix3d & rot = t.rotation();
  const Eigen::Vector3d & tran = t.translation();

  for(unsigned int i = 0; i < 3; ++i)
  {
    for(unsigned int j = 0; j < 3; ++j)
    {
      m(i, j) = rot(j, i);
    }
  }

  m(0, 3) = tran(0);
  m(1, 3) = tran(1);
  m(2, 3) = tran(2);

  obj.setTransformation(m);
}

STP_BV * STPBV(const std::string & filename)
{
  STP_BV * s = new STP_BV;
  s->constructFromFile(filename);
  return s;
}

S_Polyhedron * Polyhedron(const std::string & filename)
{
  S_Polyhedron * s = new S_Polyhedron;
  s->constructFromFile(filename);
  return s;
}

double distance(CD_Pair & pair, Eigen::Vector3d & p1, Eigen::Vector3d & p2)
{
  sch::Point3 p1Tmp, p2Tmp;
  double dist = pair.getClosestPoints(p1Tmp, p2Tmp);

  p1 << p1Tmp[0], p1Tmp[1], p1Tmp[2];
  p2 << p2Tmp[0], p2Tmp[1], p2Tmp[2];

  return dist;
}

} // namespace mc_rbdyn

} // namespace sch
