#include <mc_tvm/Convex.h>

#include <mc_tvm/RobotFrame.h>

#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/SCHAddon.h>

namespace mc_tvm
{

Convex::Convex(NewConvexToken,
               mc_rbdyn::S_ObjectPtr object,
               const mc_rbdyn::RobotFrame & frame,
               const sva::PTransformd & X_f_c)
: object_(object), frame_(frame), X_f_c_(std::move(X_f_c))
{
  auto & tvm_frame = frame.tvm_frame();
  registerUpdates(Update::Position, &Convex::updatePosition);
  addInputDependency(Update::Position, tvm_frame, Frame::Output::Position);
  addOutputDependency(Output::Position, Update::Position);
}

void Convex::updatePosition()
{
  sch::mc_rbdyn::transform(*object_, X_f_c_ * frame_->tvm_frame().position());
}

} // namespace mc_tvm
