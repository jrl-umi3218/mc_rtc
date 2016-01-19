#include <mc_rbdyn/surface_hull.h>

#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>

#include <mc_rtc/logging.h>

#include <stdlib.h>
#include <fstream>

#ifdef WIN32
#include <Windows.h>

inline int mkstemp(char * out)
{
  char tmp_dir[MAX_PATH + 1];
  GetTempPath(MAX_PATH + 1, tmp_dir);
  int ret = GetTempFileName(tmp_dir, "mkstemp", 0, out);
  if (ret == 0) { return -1; }
  else { return 0; }
}
#endif

namespace mc_rbdyn
{

sch::S_Object * surface_to_sch(const mc_rbdyn::Surface & surface, const double & depth, const unsigned int & slice)
{
  if(dynamic_cast<const mc_rbdyn::PlanarSurface *>(&surface) != 0)
  {
    return planar_hull(reinterpret_cast<const mc_rbdyn::PlanarSurface&>(surface), depth);
  }
  if(dynamic_cast<const mc_rbdyn::CylindricalSurface *>(&surface) != 0)
  {
    return cylindrical_hull(reinterpret_cast<const mc_rbdyn::CylindricalSurface&>(surface), slice);
  }
  if(dynamic_cast<const mc_rbdyn::GripperSurface *>(&surface) != 0)
  {
    return gripper_hull(reinterpret_cast<const mc_rbdyn::GripperSurface&>(surface), slice);
  }
  return 0;
}

sch::S_Object * sch_polyhedron(const std::vector<sva::PTransformd> & points)
{
#ifndef WIN32
  char qcIn[16] = "/tmp/qcINXXXXXX";
  char qcOut[17] = "/tmp/qcOUTXXXXXX";
#else
  char qcIn[MAX_PATH + 1];
  memset(qcIn, 0, MAX_PATH + 1);
  char qcOut[MAX_PATH + 1];
  memset(qcOut, 0, MAX_PATH + 1);
#endif
  int err = mkstemp(qcIn);
  if(err < 0)
  {
    LOG_ERROR("Failed to create temporary input file " << qcIn)
    return 0;
  }
  err = mkstemp(qcOut);
  if(err < 0)
  {
    LOG_ERROR("Failed to create temporary output file " << qcOut)
    return 0;
  }

  std::ofstream ofs(qcIn);
  ofs << "3" << std::endl;
  ofs << points.size() << std::endl;
  for(const sva::PTransformd & p : points)
  {
    const Eigen::Vector3d & t = p.translation();
    ofs << t(0) << " " << t(1) << " " << t(2) << std::endl;
  }
  ofs.close();

  /*FIXME Use libqhull directly for that? */
  std::stringstream ss;
  ss << "qconvex TI " << qcIn << " TO " << qcOut << " Qt o f";
  err = system(ss.str().c_str());
  if(err != 0)
  {
    LOG_ERROR("Invokation of qconvex with the following command failed: " << ss.str())
    return 0;
  }

  return sch::mc_rbdyn::Polyhedron(qcOut);
}

sch::S_Object * planar_hull(const mc_rbdyn::PlanarSurface & surface, const double & depth)
{
  std::vector<sva::PTransformd> points = surface.points();
  sva::PTransformd offset(Eigen::Vector3d(0, 0, depth));
  for(const sva::PTransformd & p : surface.points())
  {
    points.push_back(offset*p);
  }
  return sch_polyhedron(points);
}

sch::S_Object * cylindrical_hull(const mc_rbdyn::CylindricalSurface & surface, const unsigned int & slice)
{
  std::vector<sva::PTransformd> points(0);
  sva::PTransformd bTransform(Eigen::Vector3d(0, 0, surface.radius()));
  for(const sva::PTransformd & p : surface.points())
  {
    for(unsigned int s = 0; s < slice; ++s)
    {
      points.push_back(bTransform * sva::PTransformd(sva::RotX((2*M_PI*s)/slice))*p);
    }
  }
  return sch_polyhedron(points);
}

sch::S_Object * gripper_hull(const mc_rbdyn::GripperSurface & surface, const double & depth)
{
  std::vector<sva::PTransformd> points(0);
  for(const sva::PTransformd & p : surface.pointsFromOrigin())
  {
    points.push_back( sva::PTransformd(Eigen::Vector3d(p.translation().x(), p.translation().y(), fabs(p.translation().z()))) * surface.X_b_s() );
  }
  sva::PTransformd offset = sva::PTransformd(Eigen::Vector3d(depth, depth, depth));
  size_t nP = points.size();
  for(size_t i = 0; i < nP; ++i)
  {
    points.push_back(offset*points[i]);
  }
  return sch_polyhedron(points);
}

}
