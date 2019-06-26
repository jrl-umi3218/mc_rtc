/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_posture_controller.h"

#include <mc_rtc/logging.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

/* Common stuff */
MCPostureController::MCPostureController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt)
{
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->addTask(postureTask.get());
  qpsolver->setContacts({});

  postureTask->stiffness(1.0);
  LOG_SUCCESS("MCPostureController init done " << this)
  gui()->addElement({"Grippers"}, mc_rtc::gui::Form("GO",
                                                    [this](const mc_rtc::Configuration & form) {
                                                      double opening = form("Opening");
                                                      grippers.at("l_gripper")->setTargetOpening(opening);
                                                    },
                                                    mc_rtc::gui::FormNumberInput("Opening", true, 0.2)));
}

bool MCPostureController::run()
{
  return mc_control::MCController::run(mc_solver::FeedbackType::Joints);
}

} // namespace mc_control
