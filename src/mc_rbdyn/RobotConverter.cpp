/*
 * Copyright 2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotConverter.h>

namespace mc_rbdyn
{

RobotConverter::RobotConverter(const RobotConverterConfig & config) : config_(config) {}

void RobotConverter::precompute(const mc_rbdyn::Robot & inputRobot, mc_rbdyn::Robot & outputRobot)
{
  if(config_.mbcToOutMbc_)
  { // Construct list of common joints between inputRobot and outputRobot
    commonJointIndices_.reserve(std::max(inputRobot.mb().joints().size(), outputRobot.mb().joints().size()));
    for(const auto & joint : inputRobot.mb().joints())
    {
      const auto & jname = joint.name();
      if(outputRobot.hasJoint(jname)
         && outputRobot.mb().joint(static_cast<int>(outputRobot.jointIndexByName(jname))).dof() == joint.dof())
      {
        commonJointIndices_.emplace_back(inputRobot.jointIndexByName(jname), outputRobot.jointIndexByName(jname));
      }
    }
  }

  if(config_.copyEncoderValues_ || config_.copyEncoderVelocities_ || config_.encodersToOutMbc_)
  { // Common actuated joints and its index in output mbc
    commonEncoderToJointIndices_.reserve(inputRobot.refJointOrder().size());
    commonEncoderIndices_.reserve(inputRobot.refJointOrder().size());
    const auto & rjo = inputRobot.refJointOrder();
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      const auto & jname = rjo[i];
      if(outputRobot.hasJoint(jname))
      {
        const auto & oRjo = outputRobot.refJointOrder();
        auto it = std::find(oRjo.begin(), oRjo.end(), jname);
        if(it != oRjo.end())
        {
          commonEncoderIndices_.emplace_back(i, std::distance(oRjo.begin(), it));
        }

        auto mbcIndex = outputRobot.jointIndexByName(jname);
        commonEncoderToJointIndices_.emplace_back(i, mbcIndex);
      }
    }
  }

  if(config_.copyEncoderValues_)
  {
    outEncoderValues_.resize(outputRobot.refJointOrder().size());
  }

  if(config_.copyEncoderVelocities_)
  {
    outEncoderVelocities_.resize(outputRobot.refJointOrder().size());
  }

  if(config_.computeMimics_)
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

void RobotConverter::convert(const mc_rbdyn::Robot & inputRobot, mc_rbdyn::Robot & outputRobot)
{
  if(first_)
  {
    precompute(inputRobot, outputRobot);
    first_ = false;
  }

  // Copy the encoders into outputRobot
  const auto & encoders = inputRobot.encoderValues();
  if(encoders.size() == inputRobot.refJointOrder().size())
  {
    if(config_.encodersToOutMbc_)
    {
      for(const auto & indices : commonEncoderToJointIndices_)
      {
        outputRobot.mbc().q[indices.second][0] = encoders[indices.first];
      }
    }
    if(config_.copyEncoderValues_)
    {
      for(const auto & indices : commonEncoderIndices_)
      {
        outEncoderValues_[indices.second] = encoders[indices.first];
      }
      outputRobot.encoderValues(outEncoderValues_);
    }
    if(config_.copyEncoderVelocities_)
    {
      for(const auto & indices : commonEncoderIndices_)
      {
        outEncoderVelocities_[indices.second] = encoders[indices.first];
      }
      outputRobot.encoderVelocities(outEncoderVelocities_);
    }
  }

  // Copy the common mbc joints into outputRobot
  auto copyMbc = [this](const std::vector<std::vector<double>> & input, std::vector<std::vector<double>> & output) {
    for(const auto & commonIndices : commonJointIndices_)
    {
      output[commonIndices.second] = input[commonIndices.first];
    }
  };
  if(config_.mbcToOutMbc_)
  {
    if(config_.q_)
    {
      copyMbc(inputRobot.mbc().q, outputRobot.mbc().q);
    }
    if(config_.alpha_)
    {
      copyMbc(inputRobot.mbc().alpha, outputRobot.mbc().alpha);
    }
    if(config_.alphad_)
    {
      copyMbc(inputRobot.mbc().alphaD, outputRobot.mbc().alphaD);
    }
    if(config_.tau_)
    {
      copyMbc(inputRobot.mbc().jointTorque, outputRobot.mbc().jointTorque);
    }
  }

  if(config_.computeMimics_)
  {
    // Handle mimics in outputRobot
    for(const auto & mimicIndices : mimicJoints_)
    {
      const auto & m = outputRobot.mb().joint(static_cast<int>(mimicIndices.second));
      outputRobot.mbc().q[mimicIndices.second][0] =
          m.mimicMultiplier() * outputRobot.mbc().q[mimicIndices.first][0] + m.mimicOffset();
    }
  }

  if(config_.copyForceSensors_)
  {
    // Copy the force sensors into outputRobot
    for(const auto & fs : inputRobot.forceSensors())
    {
      if(outputRobot.hasForceSensor(fs.name()))
      {
        outputRobot.forceSensor(fs.name()) = fs;
      }
    }
  }

  if(config_.copyBodySensors_)
  {
    // Copy the body sensors into outputRobot
    for(const auto & bs : inputRobot.bodySensors())
    {
      outputRobot.bodySensor(bs.name()) = bs;
    }
  }

  // Copy the base position which triggers all the updates
  outputRobot.posW(inputRobot.posW());
}

} // namespace mc_rbdyn
