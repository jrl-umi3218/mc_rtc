/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/Convex.h>

#include <tvm/function/abstract/Function.h>

#include <sch/CD/CD_Pair.h>

#include <RBDyn/Jacobian.h>

namespace mc_tvm
{

/** This class implements a collision function for two given objects */
class MC_TVM_DLLAPI CollisionFunction : public tvm::function::abstract::Function
{
public:
  SET_UPDATES(CollisionFunction, Value, Velocity, Jacobian, NormalAcceleration)

  /** Constructor
   *
   * \param c1 First collision object
   *
   * \param c2 Second collision object
   *
   * \param r1Selector Joint selector for the first robot
   *
   * \param r2Selector Joint selector for the second robot
   *
   * \param dt Timestep
   *
   */
  CollisionFunction(Convex & c1,
                    Convex & c2,
                    const Eigen::VectorXd & r1Selector,
                    const Eigen::VectorXd & r2Selector,
                    double dt);

  /** Called *once* every iteration to advance the iteration counter */
  void tick();

  /** Distance between the two objects */
  inline double distance() const noexcept
  {
    return this->value()(0);
  }

  /** First convex involved in the collision */
  inline const Convex & c1() const noexcept
  {
    return *c1_;
  }

  /** Closest point on c1 in inertial frame coordinates */
  inline const Eigen::Vector3d & p1() const noexcept
  {
    return p1_;
  }

  /** Second convex involved in the collision */
  inline const Convex & c2() const noexcept
  {
    return *c2_;
  }

  /** Closest point on c2 in inertial frame coordinates */
  inline const Eigen::Vector3d & p2() const noexcept
  {
    return p2_;
  }

protected:
  /* Update functions */
  void updateValue();
  void updateVelocity();
  void updateJacobian();
  void updateNormalAcceleration();

  uint64_t iter_ = 0;
  uint64_t prevIter_ = 0;

  Convex * c1_;
  Convex * c2_;
  double dt_;

  Eigen::Vector3d p1_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d p2_ = Eigen::Vector3d::Zero();

  sch::CD_Pair pair_;

  struct ObjectData
  {
    Eigen::Vector3d nearestPoint_;
    rbd::Jacobian jac_;
    Eigen::VectorXd selector_;
  };
  std::vector<ObjectData> data_;

  Eigen::Vector3d normVecDist_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d prevNormVecDist_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d speedVec_ = Eigen::Vector3d::Zero();

  /** Intermediate computation */
  Eigen::MatrixXd fullJac_;
  Eigen::MatrixXd distJac_;
};

using CollisionFunctionPtr = std::shared_ptr<CollisionFunction>;

} // namespace mc_tvm
