/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/gui.h>
#include <mc_rtc/log/Logger.h>
#include <mc_tasks/lipm_stabilizer/ZMPCC.h>

namespace mc_tasks
{
namespace lipm_stabilizer
{
void ZMPCC::reset()
{
  error_.setZero();
  comVel_.setZero();
  comAccel_.setZero();
  comOffset_.setZero();
  integrator_.reset();
}

void ZMPCC::update(const Eigen::Vector3d & distribZMP,
                   const Eigen::Vector3d & measuredZMP,
                   const sva::PTransformd & zmpFrame,
                   double dt)
{
  if(enabled_)
  {
    error_ = distribZMP - measuredZMP;
    const Eigen::Matrix3d & R_0_c = zmpFrame.rotation();
    Eigen::Vector3d newVel =
        -R_0_c.transpose()
        * Eigen::Vector3d{config_.comAdmittance.x(), config_.comAdmittance.y(), 0}.cwiseProduct(R_0_c * error_);
    comAccel_ = (newVel - comVel_) / dt;
    comVel_ = newVel;
    integrator_.add(newVel, dt);
  }
  else
  {
    comAccel_.setZero();
    comVel_.setZero();
    integrator_.add(Eigen::Vector3d::Zero(), dt); // leak to zero
  }
  comOffset_ = integrator_.eval();
}

void ZMPCC::apply(Eigen::Vector3d & com, Eigen::Vector3d & comd, Eigen::Vector3d & comdd)
{
  com += comOffset_;
  comd += comVel_;
  comdd += comAccel_;
}

void ZMPCC::addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;
  gui.addElement(category,
                 ArrayInput("CoM admittance", {"Ax", "Ay"},
                            [this]() -> const Eigen::Vector2d & { return config_.comAdmittance; },
                            [this](const Eigen::Vector2d & a) { config_.comAdmittance = a; }),
                 NumberInput("CoM integrator leak rate [Hz]", [this]() { return integrator_.rate(); },
                             [this](double T) {
                               integrator_.rate(T);
                               config_.integratorLeakRate = T;
                             }));
}

void ZMPCC::removeFromGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category)
{
  gui.removeElement(category, "CoM admittance");
  gui.removeElement(category, "CoM integrator leak rate [Hz]");
}

void ZMPCC::addToLogger(mc_rtc::Logger & logger, const std::string & name)
{
  logger.addLogEntry(name + "_zmpcc_comAdmittance",
                     [this]() -> const Eigen::Vector2d & { return config_.comAdmittance; });
  logger.addLogEntry(name + "_zmpcc_errorZMP", [this]() -> const Eigen::Vector3d & { return error_; });
}

void ZMPCC::removeFromLogger(mc_rtc::Logger & logger, const std::string & name)
{
  logger.removeLogEntry(name + "_zmpcc_comAdmittance");
  logger.removeLogEntry(name + "_zmpcc_errorZMP");
}

} // namespace lipm_stabilizer
} // namespace mc_tasks
