/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/surface_hull.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>

#include <fstream>
#include <stdlib.h>

// Does not look nice but make sure it's not confused with system headers
#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullPoints.h"
#include "libqhullcpp/QhullVertexSet.h"

#ifdef WIN32
#  include <Windows.h>

inline int mkstemp(char * out)
{
  char tmp_dir[MAX_PATH + 1];
  GetTempPath(MAX_PATH + 1, tmp_dir);
  int ret = GetTempFileName(tmp_dir, "mkstemp", 0, out);
  if(ret == 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}
#endif

namespace mc_rbdyn
{

sch::S_Object * surface_to_sch(const mc_rbdyn::Surface & surface, const double & depth, const unsigned int & slice)
{
  if(dynamic_cast<const mc_rbdyn::PlanarSurface *>(&surface) != nullptr)
  {
    return planar_hull(static_cast<const mc_rbdyn::PlanarSurface &>(surface), depth);
  }
  if(dynamic_cast<const mc_rbdyn::CylindricalSurface *>(&surface) != nullptr)
  {
    return cylindrical_hull(static_cast<const mc_rbdyn::CylindricalSurface &>(surface), slice);
  }
  if(dynamic_cast<const mc_rbdyn::GripperSurface *>(&surface) != nullptr)
  {
    return gripper_hull(static_cast<const mc_rbdyn::GripperSurface &>(surface), slice);
  }
  return nullptr;
}

sch::S_Object * sch_polyhedron(const std::vector<sva::PTransformd> & points_pt)
{
  sch::S_Polyhedron * poly = new sch::S_Polyhedron();
  auto & poly_algo = *(poly->getPolyhedronAlgorithm());

  // Build the input for qhull
  std::vector<double> points_in;
  points_in.reserve(points_pt.size() * 3);
  for(const auto & p : points_pt)
  {
    const auto & t = p.translation();
    points_in.push_back(t.x());
    points_in.push_back(t.y());
    points_in.push_back(t.z());
  }

  // Run qhull
  orgQhull::Qhull qhull;
  qhull.runQhull("", 3, static_cast<int>(points_pt.size()), points_in.data(), "Qt");

  auto points = qhull.points();
  poly_algo.vertexes_.reserve(points.size());
  for(const auto & p : points)
  {
    auto v = new sch::S_PolyhedronVertex();
    v->setCoordinates(p.coordinates()[0], p.coordinates()[1], p.coordinates()[2]);
    v->setNumber(static_cast<unsigned int>(p.id()));
    poly_algo.vertexes_.push_back(v);
  }
  auto facets = qhull.facetList();
  poly_algo.triangles_.reserve(facets.size());
  for(const auto & f : facets)
  {
    if(!f.isGood())
    {
      continue;
    }
    sch::PolyhedronTriangle t;
    t.normal.Set(f.hyperplane().coordinates());
    t.normal.normalize();
    t.a = static_cast<unsigned int>(f.vertices()[0].point().id());
    t.b = static_cast<unsigned int>(f.vertices()[1].point().id());
    t.c = static_cast<unsigned int>(f.vertices()[2].point().id());
    auto addNeighbors = [](std::vector<sch::S_PolyhedronVertex *> & vertexes_, unsigned int a, unsigned int b,
                           unsigned int c) {
      vertexes_[a]->addNeighbor(vertexes_[b]);
      vertexes_[a]->addNeighbor(vertexes_[c]);
    };
    addNeighbors(poly_algo.vertexes_, t.a, t.b, t.c);
    addNeighbors(poly_algo.vertexes_, t.b, t.a, t.c);
    addNeighbors(poly_algo.vertexes_, t.c, t.a, t.b);
    poly_algo.triangles_.push_back(t);
  }

  for(const auto & v : poly_algo.vertexes_)
  {
    v->updateFastArrays();
  }
  poly_algo.deleteVertexesWithoutNeighbors();

  return poly;
}

sch::S_Object * planar_hull(const mc_rbdyn::PlanarSurface & surface, const double & depth)
{
  std::vector<sva::PTransformd> points = surface.points();
  sva::PTransformd offset(Eigen::Vector3d(0, 0, depth));
  for(const sva::PTransformd & p : surface.points())
  {
    points.push_back(offset * p);
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
      points.push_back(bTransform * sva::PTransformd(sva::RotX((2 * mc_rtc::constants::PI * s) / slice)) * p);
    }
  }
  return sch_polyhedron(points);
}

sch::S_Object * gripper_hull(const mc_rbdyn::GripperSurface & surface, const double & depth)
{
  std::vector<sva::PTransformd> points(0);
  for(const sva::PTransformd & p : surface.pointsFromOrigin())
  {
    points.push_back(
        sva::PTransformd(Eigen::Vector3d(p.translation().x(), p.translation().y(), fabs(p.translation().z())))
        * surface.X_b_s());
  }
  sva::PTransformd offset = sva::PTransformd(Eigen::Vector3d(depth, depth, depth));
  size_t nP = points.size();
  for(size_t i = 0; i < nP; ++i)
  {
    points.push_back(offset * points[i]);
  }
  return sch_polyhedron(points);
}

} // namespace mc_rbdyn
