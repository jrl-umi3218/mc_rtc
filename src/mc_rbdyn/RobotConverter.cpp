/*
 * Copyright 2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotConverter.h>

namespace mc_rbdyn
{

RobotConverter::RobotConverter(const RobotConverterConfig & config) : config_(config) {}

void RobotConverter::precompute(const mc_rbdyn::Robot & inputRobot, mc_rbdyn::Robot & outputRobot)
{
  // Construct list of common joints between inputRobot and outputRobot
  if(config_.mbcToOutMbc)
  {
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

  if(config_.copyEncoderValues || config_.copyEncoderVelocities || config_.encodersToOutMbc)
  {
    commonEncoderToJointIndices_.reserve(inputRobot.refJointOrder().size());
    commonEncoderIndices_.reserve(inputRobot.refJointOrder().size());
    // Construct the list of common actuated joints between the
    // inputRobot and outputRobot
    const auto & rjo = inputRobot.refJointOrder();
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      const auto & jname = rjo[i];
      if(outputRobot.hasJoint(jname))
      {
        auto mbcIndex = outputRobot.jointIndexInMBC(i);
        if(mbcIndex < 0)
        {
          continue;
        }
        commonEncoderToJointIndices_.emplace_back(i, mbcIndex);

        const auto & oRjo = outputRobot.refJointOrder();
        auto it = std::find(oRjo.begin(), oRjo.end(), rjo[i]);
        if(it != oRjo.end())
        {
          commonEncoderIndices_.emplace_back(i, std::distance(oRjo.begin(), it));
        }
      }
    }
  }

  if(config_.copyEncoderValues)
  {
    outEncoderValues_.resize(outputRobot.refJointOrder().size());
  }

  if(config_.copyEncoderVelocities)
  {
    outEncoderVelocities_.resize(outputRobot.refJointOrder().size());
  }

  if(config_.computeMimics)
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
    if(config_.encodersToOutMbc)
    {
      for(const auto & indices : commonEncoderToJointIndices_)
      {
        outputRobot.mbc().q[indices.second][0] = encoders[indices.first];
      }
    }
    if(config_.copyEncoderValues)
    {
      for(const auto & indices : commonEncoderToJointIndices_)
      {
        outEncoderValues_[indices.second] = encoders[indices.first];
      }
      outputRobot.encoderValues(outEncoderValues_);
    }
    if(config_.copyEncoderValues)
    {
      for(const auto & indices : commonEncoderToJointIndices_)
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
  if(config_.mbcToOutMbc)
  {
    if(config_.q)
    {
      copyMbc(inputRobot.mbc().q, outputRobot.mbc().q);
    }
    if(config_.alpha)
    {
      copyMbc(inputRobot.mbc().alpha, outputRobot.mbc().alpha);
    }
    if(config_.alphad)
    {
      copyMbc(inputRobot.mbc().alphaD, outputRobot.mbc().alphaD);
    }
    if(config_.tau)
    {
      copyMbc(inputRobot.mbc().jointTorque, outputRobot.mbc().jointTorque);
    }
  }

  if(config_.computeMimics)
  {
    // Handle mimics in outputRobot
    for(const auto & mimicIndices : mimicJoints_)
    {
      const auto & m = outputRobot.mb().joint(static_cast<int>(mimicIndices.second));
      outputRobot.mbc().q[mimicIndices.second][0] =
          m.mimicMultiplier() * outputRobot.mbc().q[mimicIndices.first][0] + m.mimicOffset();
    }
  }

  if(config_.copyForceSensors)
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

  if(config_.copyBodySensors)
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
