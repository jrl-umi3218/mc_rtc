/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/SCHAddon.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/pragma.h>

#include <RBDyn/FK.h>
#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

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

Robots::Robots() : robots_(), mbs_(), mbcs_(), robotIndex_(0), envIndex_(0) {}

Robots::Robots(const Robots & rhs)
: robot_modules_(rhs.robot_modules_), robots_(), mbs_(rhs.mbs_), mbcs_(rhs.mbcs_), mbgs_(rhs.mbgs_),
  robotIndex_(rhs.robotIndex_), envIndex_(rhs.envIndex_)
{
  for(unsigned int i = 0; i < rhs.robots_.size(); ++i)
  {
    const Robot & robot = rhs.robots_[i];
    robot.copy(*this, robot.name(), i);
  }
}

Robots & Robots::operator=(const Robots & rhs)
{
  if(&rhs == this)
  {
    return *this;
  }
  robots_.clear();
  robot_modules_ = rhs.robot_modules_;
  mbs_ = rhs.mbs_;
  mbcs_ = rhs.mbcs_;
  mbgs_ = rhs.mbgs_;
  robotIndex_ = rhs.robotIndex_;
  envIndex_ = rhs.envIndex_;
  for(unsigned int i = 0; i < rhs.robots_.size(); ++i)
  {
    const Robot & robot = rhs.robots_[i];
    robot.copy(*this, robot.name(), i);
  }
  return *this;
}

std::vector<Robot> & Robots::robots()
{
  return robots_;
}
const std::vector<Robot> & Robots::robots() const
{
  return robots_;
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
    mc_rtc::log::error_and_throw<std::runtime_error>("No robot named {}", name);
  }
  return key->second;
}

Robot & Robots::robot()
{
  return robots_[robotIndex_];
}
const Robot & Robots::robot() const
{
  return robots_[robotIndex_];
}

Robot & Robots::env()
{
  return robots_[envIndex_];
}
const Robot & Robots::env() const
{
  return robots_[envIndex_];
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
    mc_rtc::log::error_and_throw<std::runtime_error>("No robot with index {} ({} robots loaded)", idx, robots_.size());
  }
  return robots_[idx];
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
    mc_rtc::log::error_and_throw<std::runtime_error>("No robot named {}", name);
  }
  return robots_[key->second];
}

void Robots::createRobotWithBase(const std::string & name,
                                 Robots & robots,
                                 unsigned int robots_idx,
                                 const Base & base,
                                 const Eigen::Vector3d & baseAxis)
{
  createRobotWithBase(name, robots.robot(robots_idx), base, baseAxis);
}

void Robots::createRobotWithBase(const std::string & name,
                                 Robot & robot,
                                 const Base & base,
                                 const Eigen::Vector3d & baseAxis)
{
  if(hasRobot(name))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Cannot copy robot {} with a new base as a robot named {} already exists", robot.name(), name);
  }
  this->robot_modules_.push_back(robot.module());
  this->mbs_.push_back(robot.mbg().makeMultiBody(base.baseName, base.baseType, baseAxis, base.X_0_s, base.X_b0_s));
  this->mbcs_.emplace_back(this->mbs_.back());
  this->mbgs_.push_back(robot.mbg());
  auto robotIndex = static_cast<unsigned int>(this->mbs_.size()) - 1;
  robot.copy(*this, name, robotIndex, base);
  robotNameToIndex_[name] = robotIndex;
}

void Robots::removeRobot(const std::string & name)
{
  if(!hasRobot(name))
  {
    mc_rtc::log::error("Did not find a robot named {} to remove", name);
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
  const auto & robotName = robots_[idx].name();
  robotNameToIndex_.erase(robotName);
  robot_modules_.erase(robot_modules_.begin() + idx);
  robots_.erase(robots_.begin() + idx);
  mbs_.erase(mbs_.begin() + idx);
  mbcs_.erase(mbcs_.begin() + idx);
  mbgs_.erase(mbgs_.begin() + idx);
  for(unsigned int i = idx; i < robots_.size(); ++i)
  {
    auto & r = robots_[i];
    r.robots_idx_--;
    robotNameToIndex_[r.name()] = r.robots_idx_;
  }
}

void Robots::robotCopy(const Robot & robot, const std::string & copyName)
{
  if(hasRobot(copyName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Cannot copy robot {} to {}: a robot named {} already exists",
                                                     robot.name(), copyName, copyName);
  }
  this->robot_modules_.push_back(robot.module());
  this->mbs_.push_back(robot.mb());
  this->mbcs_.push_back(robot.mbc());
  this->mbgs_.push_back(robot.mbg());
  auto copyRobotIndex = static_cast<unsigned int>(this->mbs_.size()) - 1;
  robot.copy(*this, copyName, copyRobotIndex);
  robotNameToIndex_[copyName] = copyRobotIndex;
}

// deprecated
Robot & Robots::load(const RobotModule & module,
                     const std::string &,
                     sva::PTransformd * base,
                     const std::string & bName)
{
  return load(module.name, module, base, bName);
}

Robot & Robots::load(const RobotModule & module, sva::PTransformd * base, const std::string & bName)
{
  return load(module.name, module, base, bName);
}

Robot & Robots::load(const std::string & name,
                     const RobotModule & module,
                     sva::PTransformd * base,
                     const std::string & bName)
{
  if(hasRobot(name))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Robot names are required to be unique but a robot named {} already exists.", name);
  }
  robot_modules_.emplace_back(module);
  mbs_.emplace_back(module.mb);
  mbcs_.emplace_back(module.mbc);
  mbgs_.emplace_back(module.mbg);
  mc_rbdyn::Robot robot{name, *this, static_cast<unsigned int>(mbs_.size() - 1), true, base, bName};
  robots_.emplace_back(std::move(robot));
  robotNameToIndex_[name] = robot.robotIndex();
  updateIndexes();
  return robots_.back();
}

/*void loadPolyTorqueBoundsData(const std::string & file, Robot & robot)
{
}*/

std::shared_ptr<Robots> loadRobot(const RobotModule & module,
                                  const std::string &,
                                  sva::PTransformd * base,
                                  const std::string & baseName)
{
  return loadRobot(module, base, baseName);
}

std::shared_ptr<Robots> loadRobot(const RobotModule & module, sva::PTransformd * base, const std::string & baseName)
{
  auto robots = std::make_shared<Robots>();
  robots->load(module.name, module, base, baseName);
  return robots;
}

// deprecated
void Robots::load(const RobotModule & module,
                  const std::string &,
                  const RobotModule & envModule,
                  const std::string &,
                  sva::PTransformd * base,
                  const std::string & baseName)
{
  load(module, envModule, base, baseName);
}

void Robots::load(const RobotModule & module,
                  const RobotModule & envModule,
                  sva::PTransformd * base,
                  const std::string & baseName)
{
  load(module.name, module, base, baseName);
  load(envModule.name, envModule);
}

// deprecated
std::shared_ptr<Robots> loadRobotAndEnv(const RobotModule & module,
                                        const std::string &,
                                        const RobotModule & envModule,
                                        const std::string &,
                                        sva::PTransformd * base,
                                        const std::string & baseName)
{
  return loadRobotAndEnv(module, envModule, base, baseName);
}

std::shared_ptr<Robots> loadRobotAndEnv(const RobotModule & module,
                                        const RobotModule & envModule,
                                        sva::PTransformd * base,
                                        const std::string & baseName)
{
  auto robots = std::make_shared<Robots>();
  robots->load(module, envModule, base, baseName);
  return robots;
}

void Robots::load(const std::vector<std::shared_ptr<RobotModule>> & modules, const std::vector<std::string> &)
{
  load(modules);
}

void Robots::load(const std::vector<std::shared_ptr<RobotModule>> & modules)
{
  for(size_t i = 0; i < modules.size(); ++i)
  {
    load(*modules[i]);
  }
}

std::shared_ptr<Robots> loadRobots(const std::vector<std::shared_ptr<RobotModule>> & modules,
                                   const std::vector<std::string> &)
{
  return loadRobots(modules);
}

std::shared_ptr<Robots> loadRobots(const std::vector<std::shared_ptr<RobotModule>> & modules)
{
  auto robots = std::make_shared<Robots>();
  robots->load(modules);
  return robots;
}

Robot & Robots::loadFromUrdf(const std::string & name,
                             const std::string & urdf,
                             bool withVirtualLinks,
                             const std::vector<std::string> & filteredLinks,
                             bool fixed,
                             sva::PTransformd * base,
                             const std::string & baseName)
{
  auto res = rbd::parsers::from_urdf(urdf, fixed, filteredLinks, true, "", withVirtualLinks);

  mc_rbdyn::RobotModule module(name, res);

  return load(module, base, baseName);
}

std::shared_ptr<Robots> loadRobotFromUrdf(const std::string & name,
                                          const std::string & urdf,
                                          bool withVirtualLinks,
                                          const std::vector<std::string> & filteredLinks,
                                          bool fixed,
                                          sva::PTransformd * base,
                                          const std::string & baseName)
{
  auto robots = std::make_shared<Robots>();
  robots->loadFromUrdf(name, urdf, withVirtualLinks, filteredLinks, fixed, base, baseName);
  return robots;
}

void Robots::rename(const std::string & oldName, const std::string & newName)
{
  if(!hasRobot(oldName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Cannot rename robot: no robot named {}", oldName);
  }
  if(robotNameToIndex_.count(newName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Cannot rename robot: a robot named {} already exist", newName);
  }
  auto index = robotNameToIndex_[oldName];
  robotNameToIndex_.erase(oldName);
  robotNameToIndex_[newName] = index;
  robots_[index].name(newName);
}

void Robots::updateIndexes()
{
  /* Sets robotIndex_ to the first robot with dofs != 0 and envIndex_ to the
   * last robot with dof == 0 OR the last robot */
  for(unsigned int i = 0; i < robots_.size(); ++i)
  {
    if(robots_[i].mb().nrDof())
    {
      robotIndex_ = i;
      break;
    }
  }
  envIndex_ = static_cast<unsigned int>(robots_.size()) - 1;
  for(size_t i = robots_.size(); i > 0; --i)
  {
    if(robots_[i - 1].mb().nrDof() == 0)
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
