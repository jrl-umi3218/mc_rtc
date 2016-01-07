#ifndef _H_MCSEQSTEPS_H_
#define _H_MCSEQSTEPS_H_

#include "mc_seq_controller.h"

#include <mc_control/api.h>

namespace mc_control
{

#define CREATE_STRUCT(sName)\
struct MC_CONTROL_DLLAPI sName : public SeqStep\
{\
  virtual bool eval(MCSeqController & controller) override;\
};

CREATE_STRUCT(enter_initT)
CREATE_STRUCT(live_initT)
/* Contact branch */
CREATE_STRUCT(live_chooseContactT)
CREATE_STRUCT(enter_removeContactT)
CREATE_STRUCT(live_removeContacT)
CREATE_STRUCT(enter_moveWPT)
CREATE_STRUCT(live_moveWPT)
CREATE_STRUCT(enter_moveContactP)
CREATE_STRUCT(live_moveContactT)
CREATE_STRUCT(enter_pushContactT)
CREATE_STRUCT(live_pushContactT)
/* Move CoM branch */
CREATE_STRUCT(live_chooseCoMT)
CREATE_STRUCT(enter_moveCoMP)
CREATE_STRUCT(live_CoMOpenGripperT)
CREATE_STRUCT(enter_CoMRemoveGripperT)
CREATE_STRUCT(live_CoMRemoveGripperT)
CREATE_STRUCT(live_moveCoMT)
CREATE_STRUCT(live_CoMCloseGripperT)
/* Gripper contact branch */
CREATE_STRUCT(live_chooseGripperT)
CREATE_STRUCT(enter_openGripperP)
CREATE_STRUCT(live_openGripperP)
CREATE_STRUCT(live_openGripperT)
CREATE_STRUCT(live_openGripperNotRmT)
CREATE_STRUCT(enter_removeGripperP)
CREATE_STRUCT(live_removeGripperP)
CREATE_STRUCT(live_removeGripperT)
CREATE_STRUCT(live_removeGripperNotAddT)
CREATE_STRUCT(enter_moveGripperWPT)
CREATE_STRUCT(live_moveGripperWPT)
CREATE_STRUCT(live_moveGripperT)
CREATE_STRUCT(enter_adjustGripperP)
CREATE_STRUCT(live_adjustGripperT)
CREATE_STRUCT(enter_addGripperT)
CREATE_STRUCT(live_addGripperT)
CREATE_STRUCT(enter_removeBeforeCloseT)
CREATE_STRUCT(live_removeBeforeCloseT)
CREATE_STRUCT(enter_softCloseGripperP)
CREATE_STRUCT(live_softCloseGripperP)
CREATE_STRUCT(enter_hardCloseGripperP)
CREATE_STRUCT(live_hardCloseGripperP)
CREATE_STRUCT(enter_restoreArmGainsP)
CREATE_STRUCT(enter_contactGripperP)
CREATE_STRUCT(live_contactGripperT)

#undef CREATE_STRUCT

}

#endif
