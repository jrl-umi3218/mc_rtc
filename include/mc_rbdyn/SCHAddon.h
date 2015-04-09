#include <sch/S_Polyhedron/S_Polyhedron.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Object/S_Box.h>
#include <sch/STP-BV/STP_BV.h>
#include <sch/CD/CD_Pair.h>

#include <SpaceVecAlg/SpaceVecAlg>

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

namespace sch
{

  void transform(S_Object& obj, const sva::PTransformd& t);

  S_Object* Sphere(double radius);

  S_Object* Box(double x, double y, double z);

  S_Object* STPBV(const std::string& filename);

  S_Object* Polyhedron(const std::string& filename);

  double distance(CD_Pair& pair, Eigen::Vector3d& p1, Eigen::Vector3d& p2);

}

