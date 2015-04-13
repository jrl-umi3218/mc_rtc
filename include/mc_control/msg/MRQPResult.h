#ifndef _H_MCCONTROLMSGMRQPRESULT_H_
#define _H_MCCONTROLMSGMRQPRESULT_H_

#include <cstdint>
#include <vector>
#include <mc_control/msg/Robot.h>
#include <mc_control/msg/MRContact.h>

namespace mc_control
{

struct MRQPResultMsg
{
  std::vector<RobotMsg> robots_state;
  Eigen::VectorXd lambdaVec;
  std::vector<MRContactMsg> contacts;
  std::vector<uint16_t> contacts_lambda_begin;
};

}

#endif
