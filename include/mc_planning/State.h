/*
 * Copyright (c) 2009, @author Mitsuharu Morisawa
 *
 * AIST
 *
 * All rights reserved.
 *
 * This program is made available under the terms of the Eclipse Public License
 * v1.0 which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 */
#pragma once
#include <mc_planning/api.h>
#include <Eigen/Core>

namespace mc_planning
{

class MC_PLANNING_DLLAPI StateP
{
public:
  Eigen::Vector3d P; ///< position

  /**
     @brief constructor
  */
  StateP();

  StateP(const Eigen::Vector3d & p);

  StateP(const StateP & i_state);

  StateP & operator=(const StateP & i_state);

  friend std::ostream & operator<<(std::ostream & os, const StateP & s);

  void Initialize();

  void Copy(const StateP & i_state);
};

class MC_PLANNING_DLLAPI StatePV : virtual public StateP
{
public:
  Eigen::Vector3d V; ///< velocity

  /**
     @brief constructor
  */
  StatePV();

  StatePV(const Eigen::Vector3d & p, const Eigen::Vector3d & v);

  StatePV(const StatePV & i_state);

  StatePV & operator=(const StatePV & i_state);

  friend std::ostream & operator<<(std::ostream & os, const StatePV & s);

  void Initialize();

  void Copy(const StatePV & i_state);
};

class MC_PLANNING_DLLAPI StatePVA : virtual public StatePV
{
public:
  Eigen::Vector3d Vdot; ///< acceleration

  /**
     @brief constructor
  */
  StatePVA();

  StatePVA(const Eigen::Vector3d & p, const Eigen::Vector3d & v, const Eigen::Vector3d & a);

  StatePVA(const StatePVA & i_state);

  StatePVA & operator=(const StatePVA & i_state);

  friend std::ostream & operator<<(std::ostream & os, const StatePVA & s);

  void Initialize();

  void Copy(const StatePVA & i_state);
};
} // namespace mc_planning
