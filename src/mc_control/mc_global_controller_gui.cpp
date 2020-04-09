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
    gui->removeCategory({"Global", "Grippers"});
    for(const auto & robot : controller().robots())
    {
      const auto & gi = robot.grippersByName();
      for(const auto & g : gi)
      {
        auto g_ptr = g.second.get();
        std::vector<std::string> category = {"Global", "Grippers", robot.name(), g.first};
        gui->addElement(category, mc_rtc::gui::Button("Open", [g_ptr]() { g_ptr->setTargetOpening(1); }),
                        mc_rtc::gui::Button("Close", [g_ptr]() { g_ptr->setTargetOpening(0); }),
                        mc_rtc::gui::NumberSlider("Opening percentage", [g_ptr]() { return g_ptr->opening(); },
                                                  [g_ptr](double op) { g_ptr->setTargetOpening(op); }, 0, 1),
                        mc_rtc::gui::NumberSlider("Maximum velocity percentage",
                                                  [g_ptr]() { return g_ptr->percentVMAX(); },
                                                  [g_ptr](double op) { g_ptr->percentVMAX(op); }, 0, 1));
        category.push_back("Safety");
        gui->addElement(
            category,
            mc_rtc::gui::NumberInput(
                "Actual command diff threshold [deg]",
                [g_ptr]() { return mc_rtc::constants::toDeg(g_ptr->actualCommandDiffTrigger()); },
                [g_ptr](double deg) { g_ptr->actualCommandDiffTrigger(mc_rtc::constants::toRad(deg)); }),
            mc_rtc::gui::NumberInput("Over command limiter iterations",
                                     [g_ptr]() -> double { return g_ptr->overCommandLimitIterN(); },
                                     [g_ptr](double N) { g_ptr->overCommandLimitIterN(static_cast<unsigned int>(N)); }),
            mc_rtc::gui::NumberInput(
                "Release offset [deg]", [g_ptr]() { return mc_rtc::constants::toDeg(g_ptr->releaseSafetyOffset()); },
                [g_ptr](double deg) { g_ptr->releaseSafetyOffset(mc_rtc::constants::toRad(deg)); }));
      }
    }
    gui->removeCategory({"Global", "Change controller"});
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
