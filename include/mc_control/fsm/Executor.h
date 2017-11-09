#pragma once

#include <mc_control/api.h>

namespace mc_control
{

namespace fsm
{


/** \class Executor takes care of executing an FSM
 *
 * The executor can works in two ways:
 *
 * - managed: handles transitions through an external trigger
 *
 * - self-managed: handles transitions thanks to a TransitionMap
 *
 */
struct MC_CONTROL_DLLAPI Executor
{
};

} // namespace fsm

} // namespace mc_control
