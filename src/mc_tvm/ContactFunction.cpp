/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/ContactFunction.h>

#include <mc_tvm/Robot.h>
#include <mc_tvm/RobotFrame.h>

namespace mc_tvm
{

ContactFunction::ContactFunction(const mc_rbdyn::Frame & f1, const mc_rbdyn::Frame & f2, const Eigen::Vector6d & dof)
: tvm::function::abstract::Function(6), dof_(dof)
{
  registerUpdates(Update::Value, &ContactFunction::updateValue, Update::Derivatives,
                  &ContactFunction::updateDerivatives);

  addOutputDependency<ContactFunction>(Output::Value, Update::Value);
  addOutputDependency<ContactFunction>(Output::Velocity, Update::Derivatives);
  addOutputDependency<ContactFunction>(Output::NormalAcceleration, Update::Derivatives);
  addOutputDependency<ContactFunction>(Output::Jacobian, Update::Derivatives);

  addInternalDependency<ContactFunction>(Update::Derivatives, Update::Value);

  using ConstFramePtr = const Frame *;
  auto addRobot = [this](const mc_rbdyn::Frame & fIn, ConstFramePtr & fOut, bool & useF, rbd::Jacobian & jac) {
    fOut = &fIn.tvm_frame();
    auto fRobot = dynamic_cast<const RobotFrame *>(fOut);
    if(fRobot && fRobot->rbdJacobian().dof() > 0)
    {
      auto & tvm_frame = static_cast<mc_tvm::RobotFrame &>(fIn.tvm_frame());
      const auto & r = tvm_frame.frame().robot();
      const auto & tvm_robot = r.tvmRobot();
      useF = true;
      jac = tvm_frame.rbdJacobian();
      addInputDependency<ContactFunction>(Update::Value, tvm_frame, mc_tvm::RobotFrame::Output::Position);
      addInputDependency<ContactFunction>(Update::Derivatives, tvm_frame, mc_tvm::RobotFrame::Output::Velocity);
      addInputDependency<ContactFunction>(Update::Derivatives, tvm_frame,
                                          mc_tvm::RobotFrame::Output::NormalAcceleration);
      addInputDependency<ContactFunction>(Update::Derivatives, tvm_frame, mc_tvm::RobotFrame::Output::Jacobian);
      addVariable(tvm_robot.q(), false);
      return r.mb().nrDof();
    }
    else
    {
      useF = false;
      auto & tvm_frame = fIn.tvm_frame();
      addInputDependency<ContactFunction>(Update::Value, tvm_frame, mc_tvm::Frame::Output::Position);
      return 0;
    }
  };
  X_0_cf_ = f1.position();
  X_cf_f1_ = sva::PTransformd::Identity();
  X_cf_f2_ = f2.position() * X_0_cf_.inv();
  int maxDof = addRobot(f1, f1_, use_f1_, f1Jacobian_);
  maxDof = std::max(maxDof, addRobot(f2, f2_, use_f2_, f2Jacobian_));

  jacTmp_.resize(6, std::max(f1Jacobian_.dof(), f2Jacobian_.dof()));
  jac_.resize(6, maxDof);
}

void ContactFunction::updateValue()
{
  {
    // Update X_f1_cf_ and X_f2_cf_ according to the motion allowed by dof
    Eigen::Vector6d revDof = dof_.unaryExpr([](double a) { return a != 0.0 ? 0.0 : 1.0; });
    auto updateX_cf_f = [&](const RobotFrame & f, sva::PTransformd & X_cf_f_init) {
      const auto & X_cf_f = f.position() * X_0_cf_.inv();
      Eigen::Vector6d error = revDof.asDiagonal() * sva::transformError(X_cf_f, X_cf_f_init).vector();
      auto offset = sva::PTransformd(sva::RotX(error(0)) * sva::RotY(error(1)) * sva::RotZ(error(2)), error.tail<3>());
      X_cf_f_init = offset * X_cf_f;
    };
    if(use_f1_)
    {
      updateX_cf_f(*static_cast<const RobotFrame *>(f1_), X_cf_f1_);
    }
    if(use_f2_)
    {
      updateX_cf_f(*static_cast<const RobotFrame *>(f2_), X_cf_f2_);
    }
  }
  const auto & X_0_f1cf = X_cf_f1_.inv() * f1_->position();
  const auto & X_0_f2cf = X_cf_f2_.inv() * f2_->position();
  const auto & X_f1cf_f2cf = X_0_f2cf * X_0_f1cf.inv();
  value_.head<3>() = dof_.head<3>().asDiagonal() * sva::rotationVelocity(X_f1cf_f2cf.rotation());
  value_.tail<3>() = dof_.tail<3>().asDiagonal() * X_f1cf_f2cf.translation();
  value_ = -value_;
}

void ContactFunction::updateDerivatives()
{
  velocity_.setZero();
  normalAcceleration_.setZero();
  for(const auto & var : variables())
  {
    jacobian_[var.get()].setZero();
  }
  auto updateDerivatives = [this](const mc_tvm::RobotFrame & frame, rbd::Jacobian & jac, double sign,
                                  const sva::PTransformd & X_f_cf) {
    const auto & robot = frame.frame().robot();
    const auto & tvm_robot = robot.tvmRobot();
    const auto & mb = robot.mb();
    const auto & mbc = robot.mbc();
    const auto & NAB = robot.tvmRobot().normalAccB();

    const auto & X_0_f = X_f_cf * frame.position();
    const auto & jacMat = jac.jacobian(mb, mbc, X_0_f);
    jacTmp_.block(0, 0, 6, jac.dof()).noalias() = sign * dof_.asDiagonal() * jacMat;
    jac.fullJacobian(mb, jacTmp_.block(0, 0, 6, jac.dof()), jac_);
    jacobian_[tvm_robot.q().get()] += jac_.block(0, 0, 6, mb.nrDof());

    normalAcceleration_ +=
        sign * dof_.asDiagonal()
        * (jac.normalAcceleration(mb, mbc, NAB, X_f_cf * frame.frame().X_b_f(), sva::MotionVecd::Zero()).vector());

    velocity_ += sign * dof_.asDiagonal() * jac.velocity(mb, mbc, X_f_cf * frame.frame().X_b_f()).vector();
  };
  if(use_f1_)
  {
    updateDerivatives(*static_cast<const mc_tvm::RobotFrame *>(f1_), f1Jacobian_, 1.0, X_cf_f1_.inv());
  }
  if(use_f2_)
  {
    updateDerivatives(*static_cast<const mc_tvm::RobotFrame *>(f2_), f2Jacobian_, -1.0, X_cf_f2_.inv());
  }
}

} // namespace mc_tvm
