/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Form.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>

/** This file implements GUI elements related to the global controller instance
 *  and available for each controller */

namespace mc_control
{

void MCGlobalController::initGUI()
{
  if(controller_ && controller_->gui())
  {
    auto gui = controller_->gui();
    gui->removeCategory({"Global", "Log"});
    gui->addElement({"Global", "Log"}, mc_rtc::gui::Button("Start a new log", [this]() { this->refreshLog(); }));
    gui->removeCategory({"Global", "Change controller"});
    gui->addElement({"Global", "Change controller"},
                    mc_rtc::gui::Label("Current controller", [this]() { return current_ctrl; }),
                    mc_rtc::gui::Form(
                        "Change controller",
                        [this](const mc_rtc::Configuration & form)
                        {
                          std::string controller = form("Controller");
                          this->EnableController(controller);
                        },
                        mc_rtc::gui::FormComboInput("Controller", true, this->enabled_controllers())));
  }
}

} // namespace mc_control
