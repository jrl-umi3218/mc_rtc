/*
 * Copyright 2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotConverter.h>

namespace mc_rbdyn
{

RobotConverter::RobotConverter(const mc_rbdyn::Robot & inputRobot,
                               mc_rbdyn::Robot & outputRobot,
                               const RobotConverterConfig & config)
: config_(config)
{
  precompute(inputRobot, outputRobot);
  if(config_.encodersToOutMbcOnce_ && !config_.encodersToOutMbc_)
  {
    encodersToOutput(inputRobot, outputRobot);
  }
  convert(inputRobot, outputRobot);
}

void RobotConverter::precompute(const mc_rbdyn::Robot & inputRobot, const mc_rbdyn::Robot & outputRobot)
{
  if(config_.mbcToOutMbc_)
  { // Construct list of common joints between inputRobot and outputRobot
    commonJointIndices_.reserve(std::max(inputRobot.mb().joints().size(), outputRobot.mb().joints().size()));
    for(const auto & joint : inputRobot.mb().joints())
    {
      // Skip fixed joints in the input robot
      if(joint.dof() == 0)
      {
        continue;
      }
      // Otherwise we can copy the joint from control to canonical if it has the same dof
      const auto & jname = joint.name();
      if(outputRobot.hasJoint(jname)
         && outputRobot.mb().joint(static_cast<int>(outputRobot.jointIndexByName(jname))).dof() == joint.dof())
      {
        commonJointIndices_.emplace_back(inputRobot.jointIndexByName(jname), outputRobot.jointIndexByName(jname));
      }
    }
  }

  if(config_.encodersToOutMbc_)
  { // Common actuated joints and its index in output mbc
    commonEncoderToJointIndices_.reserve(inputRobot.refJointOrder().size());
    const auto & rjo = inputRobot.refJointOrder();
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      const auto & jname = rjo[i];
      if(!outputRobot.hasJoint(jname))
      {
        continue;
      }
      auto outputJIndex = outputRobot.jointIndexInMBC(i);
      if(outputRobot.mb().joint(outputJIndex).dof() == 1)
      {
        commonEncoderToJointIndices_.push_back({i, outputJIndex});
      }
    }
  }

  if(config_.enforceMimics_)
  {
    for(const auto & m : outputRobot.mb().joints())
    {
      if(m.isMimic())
      {
        auto mainIndex = outputRobot.jointIndexByName(m.mimicName());
        auto mimicIndex = outputRobot.jointIndexByName(m.name());
        mimicJoints_.emplace_back(mainIndex, mimicIndex);
      }
    }
  }
}

void RobotConverter::encodersToOutput(const mc_rbdyn::Robot & inputRobot, mc_rbdyn::Robot & outputRobot) const
{
  const auto & encoders = inputRobot.encoderValues();
  if(encoders.size() == inputRobot.refJointOrder().size())
  {
    for(const auto & indices : commonEncoderToJointIndices_)
    {
      outputRobot.mbc().q[indices.second][0] = encoders[indices.first];
    }
  }
}

void RobotConverter::convert(const mc_rbdyn::Robot & inputRobot, mc_rbdyn::Robot & outputRobot) const
{
  // Copy the encoders into outputRobot
  if(config_.encodersToOutMbc_)
  {
    encodersToOutput(inputRobot, outputRobot);
  }

  // Copy the common mbc joints into outputRobot
  auto copyMbc = [this](bool doit, const std::vector<std::vector<double>> & input,
                        std::vector<std::vector<double>> & output) {
    if(!doit)
    {
      return;
    }
    for(const auto & commonIndices : commonJointIndices_)
    {
      output[commonIndices.second] = input[commonIndices.first];
    }
  };
  if(config_.mbcToOutMbc_)
  {
    const auto & mbcIn = inputRobot.mbc();
    auto & mbcOut = outputRobot.mbc();
    copyMbc(config_.copyJointCommand_, mbcIn.q, mbcOut.q);
    copyMbc(config_.copyJointVelocityCommand_, mbcIn.alpha, mbcOut.alpha);
    copyMbc(config_.copyJointAccelerationCommand_, mbcIn.alphaD, mbcOut.alphaD);
    copyMbc(config_.copyJointTorqueCommand_, mbcIn.jointTorque, mbcOut.jointTorque);
  }

  if(config_.enforceMimics_)
  {
    // Handle mimics in outputRobot
    for(const auto & mimicIndices : mimicJoints_)
    {
      const auto & m = outputRobot.mb().joint(static_cast<int>(mimicIndices.second));
      outputRobot.mbc().q[mimicIndices.second][0] =
          m.mimicMultiplier() * outputRobot.mbc().q[mimicIndices.first][0] + m.mimicOffset();
    }
  }

  if(config_.copyPosWorld_)
  {
    // Copy the base position which triggers all the updates
    outputRobot.posW(inputRobot.posW());
  }
}

} // namespace mc_rbdyn
