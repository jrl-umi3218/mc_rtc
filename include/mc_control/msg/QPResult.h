#ifndef _H_MCCONTROLMSGQPRESULT_H_
#define _H_MCCONTROLMSGQPRESULT_H_

#include <cstdint>
#include <vector>
#include <mc_control/msg/Contact.h>

namespace mc_control
{

struct QPResultMsg
{
  uint16_t stance;
  std::vector<double> q;
  Eigen::VectorXd alphaDVec;
  Eigen::VectorXd lambdaVec;
  std::vector<double> torqueVec;
  std::vector<mc_control::ContactMsg> contacts;
};

}

#endif
