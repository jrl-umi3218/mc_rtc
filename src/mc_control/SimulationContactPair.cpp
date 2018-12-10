#include <mc_control/SimulationContactPair.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/Surface.h>

namespace mc_control
{

SimulationContactPair::SimulationContactPair(const std::shared_ptr<mc_rbdyn::Surface> & robotSurface,
                                             const std::shared_ptr<mc_rbdyn::Surface> & envSurface)
: robotSurface(robotSurface), envSurface(envSurface),
  robotSch(mc_rbdyn::surface_to_sch(*(robotSurface.get()), 0.005, 8)),
  envSch(mc_rbdyn::surface_to_sch(*(envSurface.get()), -0.001, 8)), pair(robotSch.get(), envSch.get())
{
}

double SimulationContactPair::update(const mc_rbdyn::Robot & robot, const mc_rbdyn::Robot & env)
{
  updateSCH(robotSch.get(), robot, robotSurface);
  updateSCH(envSch.get(), env, envSurface);
  return pair.getDistance();
}

void SimulationContactPair::updateSCH(sch::S_Object * obj,
                                      const mc_rbdyn::Robot & robot,
                                      const std::shared_ptr<mc_rbdyn::Surface> & robotSurface)
{
  sch::mc_rbdyn::transform(*obj, robot.mbc().bodyPosW[robot.bodyIndexByName(robotSurface->bodyName())]);
}

} // namespace mc_control
