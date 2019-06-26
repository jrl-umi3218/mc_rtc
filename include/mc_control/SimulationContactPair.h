/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifndef _H_SIMULATIONCONTACTPAIR_H_
#define _H_SIMULATIONCONTACTPAIR_H_

#include <mc_control/api.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/surface_hull.h>

#include <memory>

namespace mc_control
{

struct MC_CONTROL_DLLAPI SimulationContactPair
{
public:
  SimulationContactPair(const std::shared_ptr<mc_rbdyn::Surface> & robotSurface,
                        const std::shared_ptr<mc_rbdyn::Surface> & envSurface);

  double update(const mc_rbdyn::Robot & robot, const mc_rbdyn::Robot & env);

public:
  std::shared_ptr<mc_rbdyn::Surface> robotSurface;
  std::shared_ptr<mc_rbdyn::Surface> envSurface;
  std::shared_ptr<sch::S_Object> robotSch;
  std::shared_ptr<sch::S_Object> envSch;
  sch::CD_Pair pair;

private:
  void updateSCH(sch::S_Object * obj,
                 const mc_rbdyn::Robot & robot,
                 const std::shared_ptr<mc_rbdyn::Surface> & robotSurface);
};

} // namespace mc_control

#endif
