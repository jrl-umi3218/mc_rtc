#pragma once

#include <mc_rbdyn/Robot.h>

#include <mc/rtc/deprecated.hh>

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

  /** Give access to the underlying list of Robot objects */
  std::vector<Robot> & robots();
  /** Give access to the underlying list of Robot objects (const) */
  const std::vector<Robot> & robots() const;

  /** Give access to the underlying list of rbd::MultiBody objects */
  std::vector<rbd::MultiBody> & mbs();
  /** Give access to the underlying list of rbd::MultiBody objects (const) */
  const std::vector<rbd::MultiBody> & mbs() const;

  /** Give access to the underlying list of rbd::MultiBodyConfig objects */
  std::vector<rbd::MultiBodyConfig> & mbcs();
  /** Give access to the underlying list of rbd::MultiBodyConfig objects (const) */
  const std::vector<rbd::MultiBodyConfig> & mbcs() const;

  /** Index of the main robot */
  unsigned int robotIndex() const;
  /** Index of the first non-actuated robot (or the last actuated robot if no unactuated robot are loaded) */
  unsigned int envIndex() const;

  /** @name Robot(s) loading/unloading functions
   *
   * These functions allow to load or unload robot(s) from the Robots class
   *
   * @{
   */

  /** Load a single robot from a RobotModule with an optional base
   *
   * \param module The RobotModule to fetch data from for this robot
   *
   * \param base If non-null, used as the initial transformation between the base and the world
   *
   * \param bName If empty, use the "normal" base, otherwise use body bName as base
   *
   * \returns a reference to the robot that was just loaded
   */
  Robot & load(const RobotModule & module, sva::PTransformd * base = nullptr,
               const std::string& bName = "");

  /** Load a robot and an environment from RobotModule instances with an optional base
   *
   * \param module RobotModule for the robot
   *
   * \param envModule RobotModule for the environment
   *
   * \param base If non-null, used as the initial transformation between the base and the world
   *
   * \param bName If empty, use the "normal" base, otherwise use body bName as base
   *
   */
  void load(const RobotModule & module, const RobotModule & envModule,
            sva::PTransformd * base = nullptr, const std::string & bName = "");

  /** Load multiple robots from as many RobotModule instances
   *
   * \param modules List of RobotModule to load the robots from
   *
   */
  void load(const std::vector<std::shared_ptr<RobotModule>> & modules);

  Robot & loadFromUrdf(const std::string & name, const std::string & urdf, bool withVirtualLinks = true, const std::vector<std::string> & filteredLinks = {}, bool fixed = false, sva::PTransformd * base = nullptr, const std::string& baseName = "");

  void robotCopy(const Robot & robot);

  void createRobotWithBase(Robots & robots, unsigned int robots_idx, const Base & base, const Eigen::Vector3d & baseAxis = Eigen::Vector3d::UnitZ());

  void createRobotWithBase(Robot & robot, const Base & base, const Eigen::Vector3d & baseAxis = Eigen::Vector3d::UnitZ());

  void removeRobot(const std::string & name);

  void removeRobot(unsigned int idx);

  /** @name Deprecated
   *
   * These loading functions are deprecated, do not use them in your code
   *
   * @{
   */
  Robot & load(const RobotModule & module, const std::string & surfaceDir, sva::PTransformd * base = nullptr, const std::string& bName = "") MC_RTC_DEPRECATED;

  void load(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, sva::PTransformd * base = nullptr, const std::string& baseName = "") MC_RTC_DEPRECATED;

  void load(const std::vector<std::shared_ptr<RobotModule>> & modules, const std::vector<std::string> & surfaceDirs) MC_RTC_DEPRECATED;
  /** @} */
  /* End of deprecated loading functions */

  /** @} */
  /* End of Robot(s) loading/unloading functions group */

  Robot & robot();
  const Robot & robot() const;

  Robot & env();
  const Robot & env() const;

  Robot & robot(size_t idx);
  const Robot & robot(size_t idx) const;

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

  /** Number of robots
   *
   * @return The number of robots
   *
   */
  size_type size() const noexcept;

  /** Reserves space for a total number of new_cap Robots
   *
   * \param new_cap Reserve size
   *
   * Has no effect if size > new_cap
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
MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobot(const RobotModule & module, sva::PTransformd * base = nullptr, const std::string& baseName = "");

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobot(const RobotModule & module, const std::string & surfaceDir, sva::PTransformd * base = nullptr, const std::string& baseName = "") MC_RTC_DEPRECATED;

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobots(const std::vector<std::shared_ptr<RobotModule>> & modules);

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobots(const std::vector<std::shared_ptr<RobotModule>> & modules, const std::vector<std::string> & surfaceDirs) MC_RTC_DEPRECATED;

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobotAndEnv(const RobotModule & module, const RobotModule & envModule, sva::PTransformd * base = nullptr, const std::string& baseName = "");

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobotAndEnv(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, sva::PTransformd * base = nullptr, const std::string& baseName = "") MC_RTC_DEPRECATED;

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobotFromUrdf(const std::string & name, const std::string & urdf, bool withVirtualLinks = true, const std::vector<std::string> & filteredLinks = {}, bool fixed = false, sva::PTransformd * base = nullptr, const std::string& baseName = "");

}
