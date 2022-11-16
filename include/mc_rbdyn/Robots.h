/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/iterators.h>
#include <mc_rtc/shared.h>

namespace mc_rbdyn
{

struct MC_RBDYN_DLLAPI Robots : public mc_rtc::shared<Robots>
{
  friend struct Robot;
  using value_type = mc_rbdyn::Robot;

public:
  /** @name Iterators
   *
   * These functions provide an iterator interface to Robots
   *
   * @{
   */
  using iterator = mc_rtc::SharedPtrVectorIterator<Robot>;
  using const_iterator = mc_rtc::SharedPtrVectorConstIterator<Robot>;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;
  using size_type = std::vector<RobotPtr>::size_type;
  /** @} */

  /** Give access to the underlying list of RobotModule objects */
  const std::vector<mc_rbdyn::RobotModule> & robotModules() const;

  /** Give access to the underlying list of rbd::MultiBody objects */
  std::vector<rbd::MultiBody> & mbs();
  /** Give access to the underlying list of rbd::MultiBody objects (const) */
  const std::vector<rbd::MultiBody> & mbs() const;

  /** Give access to the underlying list of rbd::MultiBodyConfig objects */
  std::vector<rbd::MultiBodyConfig> & mbcs();
  /** Give access to the underlying list of rbd::MultiBodyConfig objects (const) */
  const std::vector<rbd::MultiBodyConfig> & mbcs() const;

  /** True if the given robot is part of this intance */
  bool hasRobot(const std::string & name) const;

  /** Give access to self for backward compatibility */
  inline mc_rbdyn::Robots & robots() noexcept
  {
    return *this;
  }

  /** Give access to self for backward compatibility (const) */
  inline const mc_rbdyn::Robots & robots() const noexcept
  {
    return *this;
  }

  /** Index of the main robot */
  unsigned int robotIndex() const;
  /** Index of the first non-actuated robot (or the last actuated robot if no unactuated robot are loaded) */
  unsigned int envIndex() const;
  /** Index of a robot by name, throws if the robot does not exist */
  unsigned int robotIndex(const std::string & name) const;

  /** @name Robot(s) loading/unloading functions
   *
   * These functions allow to load or unload robot(s) from the Robots class
   *
   * @{
   */

  /** Load a single robot from a RobotModule with the provided parameters
   *
   * \param name Name of the new robot. Must be unique.
   *
   * \param module The RobotModule to fetch data from for this robot
   *
   * \param params \see LoadRobotParameters for a description of the parameters
   *
   * \throws If a robot named <name> already exists
   *
   */
  Robot & load(const std::string & name, const RobotModule & module, const LoadRobotParameters & params = {});

  /** Load a single robot from a RobotModule
   *
   * Use the name in the module to load the robot
   */
  inline Robot & load(const RobotModule & module, const LoadRobotParameters & params = {})
  {
    return load(module.name, module, params);
  }

  /** Load multiple robots from as many RobotModule instances
   *
   * \param modules List of RobotModule to load the robots from
   */
  void load(const std::vector<std::shared_ptr<RobotModule>> & modules);

  /** Rename an existing robot
   *
   * \note This is generally unsafe to do late into the controller's life
   */
  void rename(const std::string & oldName, const std::string & newName);

  /** Load a robot directly from URDF content with the given parameters
   *
   * \param name Name of the robot
   *
   * \param urdf URDF content to be parsed
   *
   * \param parser_params Parameters used to parse the URDF
   *
   * \param load_params Parameters used to load the robot
   */
  Robot & loadFromUrdf(const std::string & name,
                       const std::string & urdf,
                       const rbd::parsers::ParserParameters & parser_params = {},
                       const LoadRobotParameters & load_params = {});

  void robotCopy(const Robot & robot, const std::string & copyName);

  void removeRobot(const std::string & name);

  void removeRobot(unsigned int idx);

  /** @} */
  /* End of Robot(s) loading/unloading functions group */

  const RobotModule & robotModule(size_t idx) const;

  Robot & robot();
  const Robot & robot() const;

  Robot & env();
  const Robot & env() const;

  Robot & robot(size_t idx);
  const Robot & robot(size_t idx) const;

  Robot & robot(const std::string & name);
  const Robot & robot(const std::string & name) const;

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

  /** Access via operator[]
   *
   * Provided for backward compatibility
   */
  inline mc_rbdyn::Robot & operator[](size_t idx)
  {
    assert(idx < robots_.size());
    return *robots_[idx];
  }

  /** Access via operator[] (const)
   *
   * Provided for backward compatibility
   */
  inline const mc_rbdyn::Robot & operator[](size_t idx) const
  {
    assert(idx < robots_.size());
    return *robots_[idx];
  }

  /** Access the robot at provided index
   *
   * Provided for backward compatibility
   */
  inline mc_rbdyn::Robot & at(size_t idx)
  {
    assert(idx < robots_.size());
    return *robots_.at(idx);
  }

  /** Access the robot at provided index (const)
   *
   * Provided for backward compatibility
   */
  inline const mc_rbdyn::Robot & at(size_t idx) const
  {
    assert(idx < robots_.size());
    return *robots_.at(idx);
  }

  /** Create a new Robots instance */
  inline static RobotsPtr make() noexcept
  {
    return std::make_shared<Robots>(NewRobotsToken{});
  }

  /** Create a new Robots instance with a custom deleter */
  template<class Deleter>
  inline static RobotsPtr make(Deleter deleter) noexcept
  {
    return std::shared_ptr<Robots>(new Robots{NewRobotsToken{}}, deleter);
  }

  /** Copy this instance into another instance
   *
   * \p out will be cleared and robots loaded in this instance will be copied into \p out
   */
  void copy(mc_rbdyn::Robots & out) const;

protected:
  struct NewRobotsToken
  {
  };

public:
  Robots(NewRobotsToken);

protected:
  Robots(const Robots & rhs) = delete;
  Robots & operator=(const Robots & rhs) = delete;
  Robots(Robots && robots) = delete;
  Robots & operator=(Robots && robots) = delete;

  RobotModuleVector robot_modules_;
  std::vector<RobotPtr> robots_;
  std::vector<rbd::MultiBody> mbs_;
  std::vector<rbd::MultiBodyConfig> mbcs_;
  std::vector<rbd::MultiBodyGraph> mbgs_;
  unsigned int robotIndex_;
  unsigned int envIndex_;
  void updateIndexes();
  std::unordered_map<std::string, unsigned int> robotNameToIndex_; ///< Correspondance between robot name and index
};

/* Static pendant of the loader functions to create Robots directly */
MC_RBDYN_DLLAPI RobotsPtr loadRobot(const RobotModule & module, const LoadRobotParameters & params = {});

MC_RBDYN_DLLAPI RobotsPtr loadRobot(const std::string & name,
                                    const RobotModule & module,
                                    const LoadRobotParameters & params = {});

MC_RBDYN_DLLAPI RobotsPtr loadRobots(const std::vector<std::shared_ptr<RobotModule>> & modules);

MC_RBDYN_DLLAPI RobotsPtr loadRobotAndEnv(const RobotModule & module, const RobotModule & envModule);

MC_RBDYN_DLLAPI RobotsPtr loadRobotFromUrdf(const std::string & name,
                                            const std::string & urdf,
                                            const rbd::parsers::ParserParameters & parser_params = {},
                                            const LoadRobotParameters & load_params = {});

} // namespace mc_rbdyn
