#pragma once

#include <mc_rbdyn/Robot.h>

namespace mc_control
{
  struct MCController;
}

namespace mc_rbdyn
{

struct RobotModule;

struct MC_RBDYN_DLLAPI Robots
{
  friend struct Robot;
public:
  /** @name Iterators
  *
  * These functions provide an iterator interface to Robots
  *
  * @{
  */
  typedef typename std::vector<mc_rbdyn::Robot>::iterator iterator;
  typedef typename std::vector<mc_rbdyn::Robot>::const_iterator const_iterator;
  typedef typename std::vector<mc_rbdyn::Robot>::reverse_iterator reverse_iterator;
  typedef typename std::vector<mc_rbdyn::Robot>::const_reverse_iterator const_reverse_iterator;
  typedef typename std::vector<mc_rbdyn::Robot>::size_type size_type;
  /** @} */

  Robots();
  Robots(const Robots & rhs);
  Robots & operator=(const Robots & rhs);

  std::vector<Robot> & robots();
  const std::vector<Robot> & robots() const;

  std::vector<rbd::MultiBody> & mbs();
  const std::vector<rbd::MultiBody> & mbs() const;

  std::vector<rbd::MultiBodyConfig> & mbcs();
  const std::vector<rbd::MultiBodyConfig> & mbcs() const;

  unsigned int robotIndex() const;
  unsigned int envIndex() const;

  /* Robot(s) loader functions */
  Robot & load(const RobotModule & module, const std::string & surfaceDir, sva::PTransformd * base = nullptr, const std::string& bName = "");

  void load(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir);

  void load(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, sva::PTransformd * base, const std::string& baseName);

  void load(const std::vector<std::shared_ptr<RobotModule>> & modules, const std::vector<std::string> & surfaceDirs);

  Robot & loadFromUrdf(const std::string & name, const std::string & urdf, bool withVirtualLinks = true, const std::vector<std::string> & filteredLinks = {}, bool fixed = false, sva::PTransformd * base = nullptr, const std::string& baseName = "");

  void robotCopy(const Robot & robot);

  void createRobotWithBase(Robots & robots, unsigned int robots_idx, const Base & base, const Eigen::Vector3d & baseAxis = Eigen::Vector3d::UnitZ());

  void createRobotWithBase(Robot & robot, const Base & base, const Eigen::Vector3d & baseAxis = Eigen::Vector3d::UnitZ());

  void removeRobot(const std::string & name);

  void removeRobot(unsigned int idx);

  Robot & robot();
  const Robot & robot() const;

  Robot & env();
  const Robot & env() const;

  Robot & robot(unsigned int idx);
  const Robot & robot(unsigned int idx) const;

  /** @name Iterators
  *
  * These functions provide an iterator interface to Robots
  *
  * @{
  */
  iterator begin() noexcept;
  const_iterator begin() const noexcept;
  const_iterator cbegin() const noexcept;

  iterator end() noexcept;
  const_iterator end() const noexcept;
  const_iterator cend() const noexcept;

  reverse_iterator rbegin() noexcept;
  const_reverse_iterator rbegin() const noexcept;
  const_reverse_iterator crbegin() const noexcept;

  reverse_iterator rend() noexcept;
  const_reverse_iterator rend() const noexcept;
  const_reverse_iterator crend() const noexcept;
  /** @} */

  /**
  * @return The number of robots
  */
  size_type size() const noexcept;
  /**
  * @brief Reserves space for a total numer of new_cap Robots to prevent reallocation.
  * (Does nothing if size > new_cap)
  */
  void reserve(size_type new_cap);

protected:
  std::vector<mc_rbdyn::Robot> robots_;
  std::vector<rbd::MultiBody> mbs_;
  std::vector<rbd::MultiBodyConfig> mbcs_;
  std::vector<rbd::MultiBodyGraph> mbgs_;
  unsigned int robotIndex_;
  unsigned int envIndex_;
  void updateIndexes();
};

/* Static pendant of the loader functions to create Robots directly */
MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobot(const RobotModule & module, const std::string & surfaceDir, sva::PTransformd * base = nullptr, const std::string& baseName = "");

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobots(const std::vector<std::shared_ptr<RobotModule>> & modules, const std::vector<std::string> & surfaceDirs);

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobotAndEnv(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir);

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobotAndEnv(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, sva::PTransformd * base, const std::string& baseName);

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobotFromUrdf(const std::string & name, const std::string & urdf, bool withVirtualLinks = true, const std::vector<std::string> & filteredLinks = {}, bool fixed = false, sva::PTransformd * base = nullptr, const std::string& baseName = "");

}
