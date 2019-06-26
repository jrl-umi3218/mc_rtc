/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>

/** This file implements GUI elements related to the global controller instance
 *  and available for each controller */

namespace mc_control
{

void MCGlobalController::initGUI()
{
  if(controller_ && controller_->gui())
  {
    auto gui = controller_->gui();
    gui->removeElement({"Global"}, "Log");
    gui->addElement({"Global", "Log"}, mc_rtc::gui::Button("Start a new log", [this]() { this->refreshLog(); }));
    auto addGripper = [this, gui](const std::string & gname) {
      gui->removeCategory({"Global", "Grippers", gname});
      gui->addElement(
          {"Global", "Grippers", gname},
          mc_rtc::gui::Button("Open", [this, gname]() { this->setGripperOpenPercent(gname, 1); }),
          mc_rtc::gui::Button("Close", [this, gname]() { this->setGripperOpenPercent(gname, 0); }),
          mc_rtc::gui::NumberInput("Opening percentage",
                                   [this, gname]() { return this->controller().grippers.at(gname)->opening(); },
                                   [this, gname](double op) { this->setGripperOpenPercent(gname, op); }));
    };
    for(const auto & g : controller().grippers)
    {
      addGripper(g.first);
    }
    gui->removeElement({"Global"}, "Change controller");
    gui->addElement({"Global", "Change controller"},
                    mc_rtc::gui::Label("Current controller", [this]() { return current_ctrl; }),
                    mc_rtc::gui::Form("Change controller",
                                      [this](const mc_rtc::Configuration & form) {
                                        std::string controller = form("Controller");
                                        this->EnableController(controller);
                                      },
                                      mc_rtc::gui::FormComboInput("Controller", true, this->enabled_controllers())));
  }
}

} // namespace mc_control
