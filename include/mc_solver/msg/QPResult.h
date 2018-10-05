#pragma once

#include <mc_solver/msg/Contact.h>
#include <mc_solver/msg/Robot.h>
#include <mc_solver/msg/ZMP.h>

#include <cstdint>
#include <vector>

namespace mc_solver
{

struct QPResultMsg
{
  std::vector<RobotMsg> robots_state;
  Eigen::VectorXd lambdaVec;
  std::vector<ContactMsg> contacts;
  std::vector<ZMP> zmps;
  std::vector<int> contacts_lambda_begin;
};

} // namespace mc_solver
