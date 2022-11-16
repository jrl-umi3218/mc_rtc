/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Base.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/SCHAddon.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/pragma.h>

#include <RBDyn/FK.h>
#include <RBDyn/parsers/urdf.h>

namespace
{
template<typename sch_T>
void applyTransformToSchById(const rbd::MultiBody & mb,
                             const rbd::MultiBodyConfig & mbc,
                             std::map<std::string, std::pair<std::string, std::shared_ptr<sch_T>>> & schByName)
{
  for(auto & p : schByName)
  {
    unsigned int index = static_cast<unsigned int>(mb.bodyIndexByName(p.second.first));
    sch::mc_rbdyn::transform(*(p.second.second.get()), mbc.bodyPosW[index]);
  }
}

template<typename X, typename Y>
inline void update(std::map<X, Y> & oldData, const std::map<X, Y> & nData)
{
  for(const auto & p : nData)
  {
    oldData[p.first] = p.second;
  }
}

} // namespace

namespace mc_rbdyn
{

MC_RTC_diagnostic_push
MC_RTC_diagnostic_ignored(GCC, "-Wsign-conversion", ClangOnly, "-Wshorten-64-to-32")

Robots::Robots(NewRobotsToken) : robots_(), mbs_(), mbcs_(), robotIndex_(0), envIndex_(0) {}

void Robots::copy(Robots & out) const
{
  if(&out == this)
  {
    return;
  }
  out.robots_.clear();
  out.robot_modules_ = robot_modules_;
  out.mbs_ = mbs_;
  out.mbcs_ = mbcs_;
  out.mbgs_ = mbgs_;
  out.robotIndex_ = robotIndex_;
  out.envIndex_ = envIndex_;
  for(unsigned int i = 0; i < robots_.size(); ++i)
  {
    const Robot & robot = *robots_[i];
    out.robots_.push_back(std::make_shared<Robot>(Robot::NewRobotToken{}, robot.name(), out, i, false));
    robot.copyLoadedData(*out.robots_.back());
  }
}

std::vector<rbd::MultiBody> & Robots::mbs()
{
  return mbs_;
}
const std::vector<rbd::MultiBody> & Robots::mbs() const
{
  return mbs_;
}

std::vector<rbd::MultiBodyConfig> & Robots::mbcs()
{
  return mbcs_;
}
const std::vector<rbd::MultiBodyConfig> & Robots::mbcs() const
{
  return mbcs_;
}

unsigned int Robots::robotIndex() const
{
  return robotIndex_;
}
unsigned int Robots::envIndex() const
{
  return envIndex_;
}

unsigned int Robots::robotIndex(const std::string & name) const
{
  auto key = robotNameToIndex_.find(name);
  if(key == robotNameToIndex_.end())
  {
    mc_rtc::log::error_and_throw("No robot named {}", name);
  }
  return key->second;
}

Robot & Robots::robot()
{
  return *robots_[robotIndex_];
}
const Robot & Robots::robot() const
{
  return *robots_[robotIndex_];
}

Robot & Robots::env()
{
  return *robots_[envIndex_];
}
const Robot & Robots::env() const
{
  return *robots_[envIndex_];
}

bool Robots::hasRobot(const std::string & name) const
{
  return robotNameToIndex_.count(name);
}

Robot & Robots::robot(size_t idx)
{
  return const_cast<Robot &>(static_cast<const Robots *>(this)->robot(idx));
}
const Robot & Robots::robot(size_t idx) const
{
  if(idx >= robots_.size())
  {
    mc_rtc::log::error_and_throw("No robot with index {} ({} robots loaded)", idx, robots_.size());
  }
  return *robots_[idx];
}

Robot & Robots::robot(const std::string & name)
{
  return const_cast<Robot &>(static_cast<const Robots *>(this)->robot(name));
}

const Robot & Robots::robot(const std::string & name) const
{
  auto key = robotNameToIndex_.find(name);
  if(key == robotNameToIndex_.end())
  {
    mc_rtc::log::error_and_throw("No robot named {}", name);
  }
  return *robots_[key->second];
}

void Robots::removeRobot(const std::string & name)
{
  if(!hasRobot(name))
  {
    mc_rtc::log::error("Did not find a robot named {} to remove", name);
    return;
  }
  removeRobot(robotNameToIndex_.at(name));
}

void Robots::removeRobot(unsigned int idx)
{
  if(idx >= robots_.size())
  {
    mc_rtc::log::error("Cannot remove a robot at index {} because there is {} robots loaded", idx, robots_.size());
    return;
  }
  const auto & robotName = robots_[idx]->name();
  robotNameToIndex_.erase(robotName);
  robot_modules_.erase(robot_modules_.begin() + idx);
  robots_.erase(robots_.begin() + idx);
  mbs_.erase(mbs_.begin() + idx);
  mbcs_.erase(mbcs_.begin() + idx);
  mbgs_.erase(mbgs_.begin() + idx);
  for(unsigned int i = idx; i < robots_.size(); ++i)
  {
    auto & r = *robots_[i];
    r.robots_idx_--;
    robotNameToIndex_[r.name()] = r.robots_idx_;
  }
}

void Robots::robotCopy(const Robot & robot, const std::string & copyName)
{
  if(hasRobot(copyName))
  {
    mc_rtc::log::error_and_throw("Cannot copy robot {} to {}: a robot named {} already exists", robot.name(), copyName,
                                 copyName);
  }
  this->robot_modules_.push_back(robot.module());
  this->mbs_.push_back(robot.mb());
  this->mbcs_.push_back(robot.mbc());
  this->mbgs_.push_back(robot.mbg());
  auto referenceRobots = robot.robots_;
  auto referenceIndex = robot.robots_idx_;
  auto copyRobotIndex = static_cast<unsigned int>(this->mbs_.size()) - 1;
  robots_.push_back(std::make_shared<Robot>(Robot::NewRobotToken{}, copyName, *this, copyRobotIndex, false));
  // push_back might have invalidated the reference we were given
  const auto & refRobot = *referenceRobots->robots_[referenceIndex];
  refRobot.copyLoadedData(*robots_.back());
  robotNameToIndex_[copyName] = copyRobotIndex;
}

Robot & Robots::load(const std::string & name, const RobotModule & module, const LoadRobotParameters & params)
{
  if(hasRobot(name))
  {
    mc_rtc::log::error_and_throw("Robot names are required to be unique but a robot named {} already exists.", name);
  }
  robot_modules_.emplace_back(module);
  mbs_.emplace_back(module.mb);
  mbcs_.emplace_back(module.mbc);
  mbgs_.emplace_back(module.mbg);
  robots_.push_back(std::make_shared<Robot>(Robot::NewRobotToken{}, name, *this,
                                            static_cast<unsigned int>(mbs_.size() - 1), true, params));
  robotNameToIndex_[name] = robots_.back()->robotIndex();
  updateIndexes();
  return *robots_.back();
}

RobotsPtr loadRobot(const RobotModule & module, const LoadRobotParameters & params)
{
  return loadRobot(module.name, module, params);
}

RobotsPtr loadRobot(const std::string & name, const RobotModule & module, const LoadRobotParameters & params)
{
  auto robots = Robots::make();
  robots->load(name, module, params);
  return robots;
}

RobotsPtr loadRobotAndEnv(const RobotModule & module, const RobotModule & envModule)
{
  auto robots = Robots::make();
  robots->load(module);
  robots->load(envModule);
  return robots;
}

void Robots::load(const std::vector<std::shared_ptr<RobotModule>> & modules)
{
  for(size_t i = 0; i < modules.size(); ++i)
  {
    load(*modules[i]);
  }
}

RobotsPtr loadRobots(const std::vector<std::shared_ptr<RobotModule>> & modules)
{
  auto robots = Robots::make();
  robots->load(modules);
  return robots;
}

Robot & Robots::loadFromUrdf(const std::string & name,
                             const std::string & urdf,
                             const rbd::parsers::ParserParameters & parser_params,
                             const LoadRobotParameters & load_params)
{
  auto res = rbd::parsers::from_urdf(urdf, parser_params);
  mc_rbdyn::RobotModule module(name, res);
  return load(module, load_params);
}

RobotsPtr loadRobotFromUrdf(const std::string & name,
                            const std::string & urdf,
                            const rbd::parsers::ParserParameters & parser_params,
                            const LoadRobotParameters & load_params)
{
  auto robots = Robots::make();
  robots->loadFromUrdf(name, urdf, parser_params, load_params);
  return robots;
}

void Robots::rename(const std::string & oldName, const std::string & newName)
{
  if(!hasRobot(oldName))
  {
    mc_rtc::log::error_and_throw("Cannot rename robot: no robot named {}", oldName);
  }
  if(robotNameToIndex_.count(newName))
  {
    mc_rtc::log::error_and_throw("Cannot rename robot: a robot named {} already exist", newName);
  }
  auto index = robotNameToIndex_[oldName];
  robotNameToIndex_.erase(oldName);
  robotNameToIndex_[newName] = index;
  robots_[index]->name(newName);
}

void Robots::updateIndexes()
{
  /* Sets robotIndex_ to the first robot with dofs != 0 and envIndex_ to the
   * last robot with dof == 0 OR the last robot */
  for(unsigned int i = 0; i < robots_.size(); ++i)
  {
    if(robots_[i]->mb().nrDof())
    {
      robotIndex_ = i;
      break;
    }
  }
  envIndex_ = static_cast<unsigned int>(robots_.size()) - 1;
  for(size_t i = robots_.size(); i > 0; --i)
  {
    if(robots_[i - 1]->mb().nrDof() == 0)
    {
      envIndex_ = static_cast<unsigned int>(i) - 1;
      break;
    }
  }
}

Robots::iterator Robots::begin() noexcept
{
  return robots_.begin();
}

Robots::const_iterator Robots::begin() const noexcept
{
  return robots_.begin();
}

Robots::const_iterator Robots::cbegin() const noexcept
{
  return robots_.cbegin();
}

Robots::iterator Robots::end() noexcept
{
  return robots_.end();
}

Robots::const_iterator Robots::end() const noexcept
{
  return robots_.end();
}

Robots::const_iterator Robots::cend() const noexcept
{
  return robots_.cend();
}

Robots::reverse_iterator Robots::rbegin() noexcept
{
  return robots_.rbegin();
}

Robots::const_reverse_iterator Robots::rbegin() const noexcept
{
  return robots_.rbegin();
}

Robots::const_reverse_iterator Robots::crbegin() const noexcept
{
  return robots_.crbegin();
}

Robots::reverse_iterator Robots::rend() noexcept
{
  return robots_.rend();
}

Robots::const_reverse_iterator Robots::rend() const noexcept
{
  return robots_.rend();
}

Robots::const_reverse_iterator Robots::crend() const noexcept
{
  return robots_.crend();
}

void mc_rbdyn::Robots::reserve(mc_rbdyn::Robots::size_type new_cap)
{
  robot_modules_.reserve(new_cap);
  robots_.reserve(new_cap);
  mbs_.reserve(new_cap);
  mbcs_.reserve(new_cap);
  mbgs_.reserve(new_cap);
}

mc_rbdyn::Robots::size_type mc_rbdyn::Robots::size() const noexcept
{
  return robots_.size();
}

const RobotModule & Robots::robotModule(size_t idx) const
{
  return robot_modules_[idx];
}

MC_RTC_diagnostic_pop

} // namespace mc_rbdyn
