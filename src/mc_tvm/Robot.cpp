/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/Robot.h>

namespace mc_tvm
{

static inline void updateVar(const std::vector<std::vector<double>> & value, tvm::Variable & var)
{
  Eigen::DenseIndex idx = 0;
  for(size_t i = 0; i < value.size(); ++i)
  {
    Eigen::DenseIndex size = static_cast<Eigen::DenseIndex>(value[i].size());
    if(size == 0) { continue; }
    Eigen::Map<const Eigen::VectorXd> qi(value[i].data(), size);
    var.set(idx, size, qi);
    idx += size;
  }
}

Robot::Robot(NewRobotToken, const mc_rbdyn::Robot & robot)
: robot_(robot), normalAccB_(robot.mbc().bodyAccB.size(), sva::MotionVecd::Zero()), fd_(robot.mb())
{
  limits_.ql = rbd::paramToVector(robot_.mb(), robot_.ql());
  limits_.qu = rbd::paramToVector(robot_.mb(), robot_.qu());
  limits_.vl = rbd::dofToVector(robot_.mb(), robot_.vl());
  limits_.vu = rbd::dofToVector(robot_.mb(), robot_.vu());
  limits_.al = rbd::dofToVector(robot_.mb(), robot_.al());
  limits_.au = rbd::dofToVector(robot_.mb(), robot_.au());
  limits_.jl = rbd::dofToVector(robot_.mb(), robot_.jl());
  limits_.ju = rbd::dofToVector(robot_.mb(), robot_.ju());
  limits_.tl = rbd::dofToVector(robot_.mb(), robot_.tl());
  limits_.tu = rbd::dofToVector(robot_.mb(), robot_.tu());
  limits_.tdl = rbd::dofToVector(robot_.mb(), robot_.tdl());
  limits_.tdu = rbd::dofToVector(robot_.mb(), robot_.tdu());

  com_.reset(new CoM(CoM::NewCoMToken{}, *this));

  momentum_.reset(new Momentum(Momentum::NewMomentumToken{}, *com_));
  //
  // Create TVM variables
  {
    q_ = tvm::Space(robot.mb().nrDof(), robot.mb().nrParams(), robot.mb().nrDof()).createVariable(robot.name());
    if(robot.mb().nrJoints() > 0 && robot.mb().joint(0).type() == rbd::Joint::Free)
    {
      q_fb_ = q_->subvariable(tvm::Space(6, 7, 6), "qFloatingBase");
      q_joints_ = q_->subvariable(tvm::Space(robot.mb().nrDof() - 6, robot.mb().nrParams() - 7, robot.mb().nrDof() - 6),
                                  "qJoints", tvm::Space(6, 7, 6));
    }
    else
    {
      q_fb_ = q_->subvariable(tvm::Space(0), "qFloatingBase");
      q_joints_ = q_;
    }
    Eigen::VectorXd mimicMultiplier = Eigen::VectorXd(0);
    std::unordered_map<size_t, tvm::VariablePtr> mimicLeaders;
    std::unordered_map<size_t, mimic_variables_t> mimicFollowers;
    const auto & joints = robot.mb().joints();
    // Returns true if the provided joint is a leader in a mimic relationship
    auto isMimicLeader = [&](std::string_view jName)
    {
      return std::find_if(robot.mb().joints().begin(), robot.mb().joints().end(),
                          [&](const auto & j) { return j.isMimic() && j.mimicName() == jName; })
             != robot.mb().joints().end();
    };
    // Returns a mimic block, i.e. following a given mimic joint all joints that follow that are also mimic of the same
    // joint
    auto getMimicBlock = [&](size_t startIdx) -> std::tuple<int, int, size_t>
    {
      const auto & mimicJ = joints[startIdx];
      size_t endIdx = startIdx;
      int params = mimicJ.params();
      int dof = mimicJ.dof();
      for(size_t i = startIdx + 1; i < joints.size(); ++i)
      {
        const auto & j = joints[i];
        if(j.isMimic() && j.mimicName() == mimicJ.mimicName())
        {
          endIdx = i;
          params += j.params();
          dof += j.dof();
        }
        else { break; }
      }
      return {params, dof, endIdx};
    };
    int nParams = 0;
    int nDof = 0;
    for(size_t i = 0; i < joints.size(); ++i)
    {
      const auto & j = joints[i];
      if(isMimicLeader(j.name()))
      {
        tvm::VariablePtr var = qJoint(i);
        mimicLeaders[i] = var;
      }
      else if(j.isMimic())
      {
        auto [params, dof, endIdx] = getMimicBlock(i);
        auto leaderIdx = robot.jointIndexByName(j.mimicName());
        tvm::VariablePtr var =
            q_->subvariable(tvm::Space(dof, params), fmt::format("{}...{}", j.name(), joints[endIdx].name()),
                            tvm::Space(nDof, nParams));
        if(!mimicFollowers.count(leaderIdx)) { mimicFollowers[leaderIdx].second.resize(0); }
        mimicFollowers[leaderIdx].first.add(var);
        auto & mult = mimicFollowers[leaderIdx].second;
        auto newM = Eigen::VectorXd(mult.size() + var->size());
        newM.head(mult.size()) = mult;
        for(size_t ii = i; ii <= endIdx; ++ii)
        {
          newM(static_cast<Eigen::DenseIndex>(static_cast<size_t>(mult.size()) + ii - i)) =
              joints[ii].mimicMultiplier();
        }
        mult = newM;
        nParams += params;
        nDof += dof;
        i = endIdx;
        continue;
      }
      nParams += j.params();
      nDof += j.dof();
    }
    for(auto & m : mimicLeaders) { mimics_[m.second] = mimicFollowers[m.first]; }
  }
  tau_ = tvm::Space(robot.mb().nrDof()).createVariable(robot.name() + "_tau");
  dq_ = tvm::dot(q_, 1);
  ddq_ = tvm::dot(q_, 2);
  Eigen::VectorXd q_init = q_->value();
  rbd::paramToVector(robot.mbc().q, q_init);
  q_->set(q_init);
  dq_->setZero();
  ddq_->setZero();
  tau_->setZero();
  tau_ext_ = Eigen::VectorXd::Zero(robot.mb().nrDof());
  ddq_ext_ = Eigen::VectorXd::Zero(robot.mb().nrDof());

  const auto & rjo = robot.refJointOrder();
  refJointIndexToQIndex_.resize(rjo.size());
  refJointIndexToQDotIndex_.resize(rjo.size());
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    const auto & jN = rjo[i];
    if(robot.hasJoint(jN))
    {
      auto jIndex = robot.mb().jointIndexByName(jN);
      refJointIndexToQIndex_[i] = robot.mb().joint(jIndex).dof() != 0 ? robot.mb().jointPosInParam(jIndex) : -1;
      refJointIndexToQDotIndex_[i] = robot.mb().joint(jIndex).dof() != 0 ? robot.mb().jointPosInDof(jIndex) : -1;
    }
    else
    {
      refJointIndexToQIndex_[i] = -1;
      refJointIndexToQDotIndex_[i] = -1;
    }
  }

  /** Signal setup */
  registerUpdates(Update::FK, &Robot::updateFK, Update::FV, &Robot::updateFV, Update::FA, &Robot::updateFA,
                  Update::NormalAcceleration, &Robot::updateNormalAcceleration, Update::H, &Robot::updateH, Update::C,
                  &Robot::updateC, Update::ExternalForces, &Robot::updateEF);
  /** Output dependencies setup */
  addOutputDependency(Output::FK, Update::FK);
  addOutputDependency(Output::FV, Update::FV);
  addOutputDependency(Output::FA, Update::FA);
  addOutputDependency(Output::NormalAcceleration, Update::NormalAcceleration);
  addOutputDependency(Output::H, Update::H);
  addOutputDependency(Output::C, Update::C);
  addOutputDependency(Output::FV, Update::FV);
  addOutputDependency(Output::ExternalForces, Update::ExternalForces);
  /** Internal dependencies setup */
  addInternalDependency(Update::FV, Update::FK);
  addInternalDependency(Update::H, Update::FV);
  addInternalDependency(Update::C, Update::FV);
  addInternalDependency(Update::FA, Update::FV);
  addInternalDependency(Update::NormalAcceleration, Update::FV);
  addInternalDependency(Update::ExternalForces, Update::H);
  addInternalDependency(Update::ExternalForces, Update::FA);
}

void Robot::updateFK()
{
  updateVar(robot_.mbc().q, *q_);
}

void Robot::updateFV()
{
  updateVar(robot_.mbc().alpha, *dq_);
}

void Robot::updateFA()
{
  updateVar(robot_.mbc().alphaD, *ddq_);
  updateVar(robot_.mbc().jointTorque, *tau_);
}

void Robot::updateNormalAcceleration()
{
  if(robot_.mb().nrDof() == 0) { return; }
  const auto & pred = robot_.mb().predecessors();
  const auto & succ = robot_.mb().successors();
  for(int i = 0; i < robot_.mb().nrJoints(); ++i)
  {
    const auto & X_p_i = robot_.mbc().parentToSon[static_cast<size_t>(i)];
    const auto & vj_i = robot_.mbc().jointVelocity[static_cast<size_t>(i)];
    const auto & vb_i = robot_.mbc().bodyVelB[static_cast<size_t>(i)];
    auto succ_i = static_cast<size_t>(succ[static_cast<size_t>(i)]);
    auto pred_i = pred[static_cast<size_t>(i)];
    normalAccB_[succ_i] = vb_i.cross(vj_i);
    if(pred_i != -1) { normalAccB_[succ_i] += X_p_i * normalAccB_[static_cast<size_t>(pred_i)]; }
  }
}

void Robot::updateH()
{
  fd_.computeH(robot_.mb(), robot_.mbc());
}

void Robot::updateC()
{
  fd_.computeC(robot_.mb(), robot_.mbc());
}

tvm::VariablePtr Robot::qJoint(size_t jIdx)
{
  const auto & mb = robot_.mb();
  const auto & name_ = robot_.name();
  if(jIdx > mb.joints().size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("{} has no joint at index {}", name_, jIdx);
  }
  if(mb.joints()[jIdx].dof() == 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("{} is not actuated in {}", mb.joints()[jIdx].name(), name_);
  }
  auto offsetParam = mb.jointPosInParam(static_cast<int>(jIdx));
  auto offsetDof = mb.jointPosInDof(static_cast<int>(jIdx));
  const auto & j = mb.joints()[jIdx];
  return q_->subvariable(tvm::Space(j.dof(), j.params(), j.dof()), j.name(),
                         tvm::Space(offsetDof, offsetParam, offsetDof));
}

void Robot::updateEF()
{
  if(robot_.hasDevice<mc_rbdyn::ExternalTorqueSensor>("externalTorqueSensor"))
  {
    tau_ext_ = robot_.device<mc_rbdyn::ExternalTorqueSensor>("externalTorqueSensor").torques();
  }
  ddq_ext_ = H().inverse() * tau_ext_;
}

} // namespace mc_tvm
