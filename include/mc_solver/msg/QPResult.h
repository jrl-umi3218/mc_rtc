#pragma once

#include <cstdint>
#include <vector>
#include <mc_solver/msg/Robot.h>
#include <mc_solver/msg/Contact.h>

namespace mc_solver
{

struct QPResultMsg
{
  std::vector<RobotMsg> robots_state;
  Eigen::VectorXd lambdaVec;
  std::vector<ContactMsg> contacts;
  std::vector<int> contacts_lambda_begin;
};

}
