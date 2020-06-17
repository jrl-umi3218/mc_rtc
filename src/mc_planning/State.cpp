/*
 * Copyright (c) 2009,
 * @author Mitsuharu Morisawa
 *
 * AIST
 *
 * All rights reserved.
 *
 * This program is made available under the terms of the Eclipse Public License
 * v1.0 which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 */
#include <mc_planning/State.h>
#include <iostream>
// #include <mc_planning/MathFunction.h>
// #include <Math/Physics.h>
//

namespace mc_planning
{

////////////////////////////////////////////////////////////
StateP::StateP()
{
  StateP::Initialize();
}

StateP::StateP(const Eigen::Vector3d & p) : P(p) {}

StateP::StateP(const StateP & i_state)
{
  StateP::Copy(i_state);
}

StateP & StateP::operator=(const StateP & i_state)
{
  StateP::Copy(i_state);
  return (*this);
}

std::ostream & operator<<(std::ostream & os, const StateP & s)
{
  os << "P = " << s.P.transpose() << std::endl;
  ;
  return os;
}

void StateP::Initialize()
{
  P.setZero();
}

void StateP::Copy(const StateP & i_state)
{
  P = i_state.P;
}

////////////////////////////////////////////////////////////
StatePV::StatePV()
{
  StatePV::Initialize();
}

StatePV::StatePV(const Eigen::Vector3d & p, const Eigen::Vector3d & v) : StateP(p), V(v) {}

StatePV::StatePV(const StatePV & i_state)
{
  StatePV::Copy(i_state);
}

StatePV & StatePV::operator=(const StatePV & i_state)
{
  StatePV::Copy(i_state);
  return (*this);
}

std::ostream & operator<<(std::ostream & os, const StatePV & s)
{
  os << "P = " << s.P.transpose() << std::endl;
  os << "V = " << s.V.transpose() << std::endl;
  ;
  return os;
}

void StatePV::Initialize()
{
  StateP::Initialize();
  V.setZero();
}

void StatePV::Copy(const StatePV & i_state)
{
  StateP::Copy(i_state);
  V = i_state.V;
}

////////////////////////////////////////////////////////////
StatePVA::StatePVA()
{
  StatePVA::Initialize();
}

StatePVA::StatePVA(const Eigen::Vector3d & p, const Eigen::Vector3d & v, const Eigen::Vector3d & a)
: StatePV(p, v), Vdot(a)
{
}

StatePVA::StatePVA(const StatePVA & i_state)
{
  StatePVA::Copy(i_state);
}

StatePVA & StatePVA::operator=(const StatePVA & i_state)
{
  StatePVA::Copy(i_state);
  return (*this);
}

std::ostream & operator<<(std::ostream & os, const StatePVA & s)
{
  os << "P = " << s.P.transpose() << std::endl;
  os << "V = " << s.V.transpose() << std::endl;
  os << "Vdot = " << s.Vdot.transpose() << std::endl;
  return os;
}

void StatePVA::Initialize()
{
  StatePV::Initialize();
  Vdot.setZero();
}

void StatePVA::Copy(const StatePVA & i_state)
{
  StatePV::Copy(i_state);
  Vdot = i_state.Vdot;
}
} // namespace mc_planning
