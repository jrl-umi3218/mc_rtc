#ifndef _H_MCRBDYNSURFACEHULL_H_
#define _H_MCRBDYNSURFACEHULL_H_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Core>
#pragma GCC diagnostic pop
#include <SpaceVecAlg/SpaceVecAlg>

#include <mc_rbdyn/surface.h>
#include <mc_rbdyn/SCHAddon.h>

namespace mc_rbdyn
{

sch::S_Object * surface_to_sch(const mc_rbdyn::Surface & surface, const double & depth = 0.01, const unsigned int & slice = 8);

sch::S_Object * sch_polyhedron(const std::vector<sva::PTransformd> & points);

sch::S_Object * planar_hull(const mc_rbdyn::PlanarSurface & surface, const double & depth);

sch::S_Object * cylindrical_hull(const mc_rbdyn::CylindricalSurface & surface, const unsigned int & slice);

sch::S_Object * gripper_hull(const mc_rbdyn::GripperSurface & surface, const double & depth);

}

#endif
