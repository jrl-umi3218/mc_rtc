#ifndef _H_MCSEQSTEPS_H_
#define _H_MCSEQSTEPS_H_

#include <mc_control/mc_seq_controller.h>

namespace mc_control
{

#define CREATE_STRUCT(sName)\
struct sName : public SeqStep\
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
CREATE_STRUCT(enter_closeGripperP)
CREATE_STRUCT(live_closeGripperP)
CREATE_STRUCT(live_closeGripperT)
CREATE_STRUCT(live_closeGripperMoveCoMT)
CREATE_STRUCT(enter_contactGripperP)
CREATE_STRUCT(live_contactGripperT)

#undef CREATE_STRUCT

}

#endif
