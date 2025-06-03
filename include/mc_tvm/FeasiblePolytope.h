/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>
#include <mc_tvm/fwd.h>

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rbdyn/fwd.h>

#include <Eigen/Core>

#include <tvm/graph/abstract/Node.h>

namespace mc_tvm
{

/** A Feasible Polytope is a set of planes that can be used as a constraint
 * for a variable. It is created from an mc_rbdyn::Contact.
 *
 * Outputs:
 * - Normals: Matrix of the planes normals
 * - Offsets: Vector of the planes offsets
 */
struct MC_TVM_DLLAPI FeasiblePolytope : public tvm::graph::abstract::Node<FeasiblePolytope>
{
  friend struct mc_rbdyn::Contact;

private:
  struct NewPolytopeToken
  {
  };

public:
  SET_OUTPUTS(FeasiblePolytope, Polytope)
  SET_UPDATES(FeasiblePolytope, Polytope)

  /**
   * @brief Construct a new Feasible Polytope object
   *
   * @param contact rbdyn contact associated
   */
  FeasiblePolytope(NewPolytopeToken, const mc_rbdyn::Contact & contact, const int & rIndex);

  FeasiblePolytope(const FeasiblePolytope &) = delete;
  FeasiblePolytope & operator=(const FeasiblePolytope &) = delete;

  /** Access the normals matrix of the polytope */
  inline const Eigen::MatrixXd & normals() const noexcept { return normals_; }

  /** Access the offsets vector of the polytope */
  inline const Eigen::VectorXd & offsets() const noexcept { return offsets_; }

  /** Access the associated contact */
  inline const mc_rbdyn::Contact & contact() const noexcept { return contact_; }

  void updatePolytope();

private:
  /** Parent instance */
  const mc_rbdyn::Contact & contact_;

  /** Robot index for this polytope */
  const int rIndex_;

  /** Set of planes */
  Eigen::MatrixXd normals_;
  Eigen::VectorXd offsets_;

  // Function to generate 3d normals of a friction cone (not the generating rays)
  Eigen::MatrixX3d generatePolyhedralConeHRep(int numberOfFrictionSides,
                                              Eigen::Matrix3d rotX_r1_r2,
                                              double m_frictionCoef)
  {
    Eigen::MatrixX3d HRep(numberOfFrictionSides, 3);
    Eigen::Vector3d contactNormal(Eigen::Vector3d::UnitZ());
    Eigen::Vector3d tan(Eigen::Vector3d::UnitX());

    // The angle to the contact normal of the friction cone is atan(mu)
    // (mu for external approximation, mu/sqrt(2) for internal, let's pick internal for H-rep)
    // But here for hrep we want the normals of the linearized cone's faces
    // --> there is a 90° angle to add to get the face normal
    double angle = (M_PI / 2.) + atan(m_frictionCoef / sqrt(2));
    // This is the first face normal
    Eigen::Vector3d normal = Eigen::AngleAxisd(angle, tan) * contactNormal;

    // step is the scale decomposition (precision) with which to compute the actual cone (linearization)
    double step = (M_PI * 2.) / numberOfFrictionSides;

    // here we compute the hrep: the rows will be the normals of the cone faces in the controlled frame
    for(int i = 0; i < numberOfFrictionSides; i++)
    {
      // Rotation around contact normal for each decomposed angle, then transposed in the relative contact frame
      HRep.row(i) = rotX_r1_r2.transpose() * Eigen::AngleAxisd(step * i, contactNormal) * normal;
    }

    return HRep;
  }

  // Function to generate 6D CoP and rotational friction constraints from the points of a rectangular surface contact
  Eigen::MatrixXd computeSurfaceTorqueConstraint(const mc_rbdyn::Surface & surface, double frictionCoeff)
  {
    const auto & surfacePoints = surface.points();
    // Using the inner friction coeff approximation (linearization)
    const auto mu = frictionCoeff / sqrt(2);
    // Find boundaries in surface frame along the surface's sagital (x) and lateral (y) direction
    double minSagital = std::numeric_limits<double>::max();
    double minLateral = std::numeric_limits<double>::max();
    double maxSagital = -std::numeric_limits<double>::max();
    double maxLateral = -std::numeric_limits<double>::max();

    Eigen::Vector3d surfaceCenter(0., 0., 0.);
    for(const auto & point : surfacePoints)
    {
      // Points are defined in body frame, convert to surface frame
      Eigen::Vector3d surfacePoint = surface.X_b_s().rotation() * (point.translation() - surface.X_b_s().translation());
      double x = surfacePoint.x();
      double y = surfacePoint.y();
      minSagital = std::min(minSagital, x);
      maxSagital = std::max(maxSagital, x);
      minLateral = std::min(minLateral, y);
      maxLateral = std::max(maxLateral, y);
      surfaceCenter += surfacePoint;
    }
    // Getting the surface center (s) in the contact surface frame (c)
    surfaceCenter /= double(surfacePoints.size());
    sva::PTransformd X_c_s = sva::PTransformd(surfaceCenter);

    // Getting half-width Y and half-length X
    auto X = (maxSagital - minSagital) / 2.;
    auto Y = (maxLateral - minLateral) / 2.;

    // The CoP constraints on tau x and tau y are from equations 18 and 19 of
    // <https://hal.archives-ouvertes.fr/hal-02108449/document>
    // The tau z constraints are the 8 derived inequalities from equation 20
    // The other ones come from the friction cone and are already taken into account in the polytope

    // Assuming this is a rectangular contact
    Eigen::Matrix<double, 12, 6> centeredSurfaceConstraintMat;
    // clang-format off
    centeredSurfaceConstraintMat <<
    // mx,  my,  mz,  fx,  fy,            fz,
       -1,   0,   0,   0,   0,            -Y,
       +1,   0,   0,   0,   0,            -Y,
        0,  -1,   0,   0,   0,            -X,
        0,  +1,   0,   0,   0,            -X,
      +mu, +mu,  -1,  -Y,  -X, -(X + Y) * mu,
      +mu, -mu,  -1,  -Y,  +X, -(X + Y) * mu,
      -mu, +mu,  -1,  +Y,  -X, -(X + Y) * mu,
      -mu, -mu,  -1,  +Y,  +X, -(X + Y) * mu,
      +mu, +mu,  +1,  +Y,  +X, -(X + Y) * mu,
      +mu, -mu,  +1,  +Y,  -X, -(X + Y) * mu,
      -mu, +mu,  +1,  -Y,  +X, -(X + Y) * mu,
      -mu, -mu,  +1,  -Y,  -X, -(X + Y) * mu;
    // clang-format on

    // Transform constraints from surface center to contact frame
    Eigen::Matrix<double, 12, 6> surfaceConstraintMat = centeredSurfaceConstraintMat * X_c_s.dualMatrix();

    return surfaceConstraintMat;
  }
};

} // namespace mc_tvm
