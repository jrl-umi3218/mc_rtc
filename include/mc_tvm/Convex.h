/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>
#include <mc_tvm/fwd.h>

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/fwd.h>

#include <tvm/graph/abstract/Node.h>

namespace mc_tvm
{

/** A convex is an SCH object associated to a Frame
 *
 * It defines a single output:
 * - Position: update the convex position according to the frame
 */
struct MC_TVM_DLLAPI Convex : public tvm::graph::abstract::Node<Convex>
{
  friend struct mc_rbdyn::Robot;

private:
  struct NewConvexToken
  {
  };

public:
  SET_OUTPUTS(Convex, Position)
  SET_UPDATES(Convex, Position)

  /** Construct a convex from associated S_Object and frame
   *
   * \param object Pointer to the loaded S_Object
   *
   * \param frame Frame of this convex
   *
   * \param X_f_c Transformation from the frame to the convex
   */
  Convex(NewConvexToken,
         mc_rbdyn::S_ObjectPtr object,
         const mc_rbdyn::RobotFrame & frame,
         const sva::PTransformd & X_f_c);

  Convex(const Convex &) = delete;
  Convex & operator=(const Convex &) = delete;

  /** Access the underlying SCH object */
  inline mc_rbdyn::S_ObjectPtr convex() noexcept
  {
    return object_;
  }

  /** Access the associated frame */
  inline const mc_rbdyn::RobotFrame & frame()
  {
    return *frame_;
  }

private:
  mc_rbdyn::S_ObjectPtr object_;
  mc_rbdyn::ConstRobotFramePtr frame_;
  sva::PTransformd X_f_c_;

  void updatePosition();
};

} // namespace mc_tvm
