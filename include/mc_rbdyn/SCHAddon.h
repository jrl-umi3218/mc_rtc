#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

#include <sch/CD/CD_Pair.h>
#include <sch/STP-BV/STP_BV.h>
#include <sch/S_Object/S_Box.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Polyhedron/S_Polyhedron.h>

// This file is part of sch-core-python.
//
// sch-core-python is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// sch-core-python is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with sch-core-python.  If not, see <http://www.gnu.org/licenses/>.

#include <mc_rbdyn/api.h>

namespace sch
{

namespace mc_rbdyn
{

MC_RBDYN_DLLAPI void transform(S_Object & obj, const sva::PTransformd & t);

MC_RBDYN_DLLAPI STP_BV * STPBV(const std::string & filename);

MC_RBDYN_DLLAPI S_Polyhedron * Polyhedron(const std::string & filename);

MC_RBDYN_DLLAPI double distance(CD_Pair & pair, Eigen::Vector3d & p1, Eigen::Vector3d & p2);

} // namespace mc_rbdyn

} // namespace sch
