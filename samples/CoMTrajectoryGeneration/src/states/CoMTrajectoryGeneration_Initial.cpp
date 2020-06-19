#include "CoMTrajectoryGeneration_Initial.h"
#include "../CoMTrajectoryGeneration.h"
#include <mc_rtc/io_utils.h>
#include <mc_rtc/time_utils.h>

namespace mc_samples
{

void CoMTrajectoryGeneration_Initial::configure(const mc_rtc::Configuration & config)
{
  config("previewTime", previewTime_);
}

void CoMTrajectoryGeneration_Initial::start(mc_control::fsm::Controller & ctl)
{
  m_dt = ctl.timeStep;
  previewSize_ = static_cast<unsigned>(lround(previewTime_ / m_dt));
  /// XXX should this be CoM height or waist height?
  double waist_height = ctl.realRobot().com().z();

  leftFootPos_ = ctl.robot().surfacePose("LeftFootCenter").translation();
  rightFootPos_ = ctl.robot().surfacePose("RightFootCenter").translation();
  feetCenterPos_ = (leftFootPos_ + rightFootPos_)/2;

  double leftY = leftFootPos_.y();
  double rightY = rightFootPos_.y();
  double centerY = feetCenterPos_.y();
  //clang-format off
  m_steps =
  {
    {0.0, 0.0, 0.0},
    {2.0, 0.0, 0.0},
    {3.5, 0.0, rightY},
    {4,   0.0, rightY},
    {5.5, 0.0, leftY},
    {7,   0.0, rightY},
    {9,   0.0, centerY},
    {10,  0.0, centerY}
  };
  //clang-format on

  comGenerator_ = std::make_shared<mc_planning::generator>(previewSize_, m_dt, ctl.robot().mass(),
                                                           waist_height);
  comGenerator_->addToLogger(ctl.logger());
  comGenerator_->steps(m_steps);
  mc_rtc::log::info(
      "Desired steps:\nTime\tCoM X\tCoM Y\n{}",
      mc_rtc::io::to_string(m_steps, [](const Eigen::Vector3d & v) -> Eigen::RowVector3d { return v; }, "\n"));

  using Color = mc_rtc::gui::Color;
  using Style = mc_rtc::gui::plot::Style;
  ctl.gui()->addPlot(
  "Trajectory (Y)",
  mc_rtc::gui::plot::X("t", [this]() { return t_; }),
  mc_rtc::gui::plot::Y("Output CoM", [this]() { return comGenerator_->OutputCOGPosition().y(); }, Color::Green),
  mc_rtc::gui::plot::Y("Ideal  CoM", [this]() { return comGenerator_->IdealCOGPosition().y(); }, Color::Green, Style::Dashed),
  mc_rtc::gui::plot::Y("Output ZMP", [this]() { return comGenerator_->OutputZMPPosition().y(); }, Color::Blue),
  mc_rtc::gui::plot::Y("Ideal  ZMP", [this]() { return comGenerator_->IdealZMPPosition().y(); }, Color::Blue, Style::Dashed));

  ctl.gui()->addElement({"CoMTrajectoryGeneration"},
  mc_rtc::gui::Button("Left",
                      [this]()
                      {
                        const auto & curr = t_;
                        m_steps.clear();
                        m_steps.push_back(Eigen::Vector3d{curr+3, 0, leftFootPos_.y()});
                        comGenerator_->steps(m_steps);
                        updateSteps();
                      }),
  mc_rtc::gui::Button("Right",
                      [this]()
                      {
                        const auto & curr = t_;
                        m_steps.clear();
                        m_steps.push_back(Eigen::Vector3d{curr+3, 0, rightFootPos_.y()});
                        updateSteps();
                      }),
  mc_rtc::gui::Button("Center",
                      [this]()
                      {
                        const auto & curr = t_;
                        m_steps.clear();
                        m_steps.push_back(Eigen::Vector3d{curr+3, 0, feetCenterPos_.y()});
                        updateSteps();
                        })
  );

  ctl.logger().addLogEntry("perf_CoMGenerator[ms]",
                           [this]()
                           {
                            return generationTime_;
                           });
}

void CoMTrajectoryGeneration_Initial::updateSteps()
{
  // Repeat last element
  m_steps.push_back(m_steps.back());
  m_steps.back()(0) += previewTime_;
  comGenerator_->steps(m_steps);
  mc_rtc::log::info("Current time: {}", t_);
  mc_rtc::log::info(
      "Desired steps:\nTime\tCoM X\tCoM Y\n{}",
      mc_rtc::io::to_string(comGenerator_->steps(), [](const Eigen::Vector3d & v) -> Eigen::RowVector3d { return v; }, "\n"));
}

bool CoMTrajectoryGeneration_Initial::run(mc_control::fsm::Controller & ctl)
{
  if(t_ < m_steps.back()(0))
  {
    auto generateTrajectory = [this]()
    {
      comGenerator_->generate(iter_);
    };
    generationTime_ = mc_rtc::measure_ms::execution(generateTrajectory).count();

    if(ctl.datastore().has("StabilizerStandingTrackCoM::target"))
    {
      ctl.datastore().call("StabilizerStandingTrackCoM::target", comGenerator_->OutputCOGPosition(), comGenerator_->OutputCOGVelocity(), comGenerator_->OutputCOGAcceleration(), comGenerator_->OutputZMPPosition());
    }
  }
  else
  {
    // mc_rtc::log::info("No more target");
    generationTime_ = 0;
  }

  iter_++;
  t_ += m_dt;
  output("OK");
  return false;
}

void CoMTrajectoryGeneration_Initial::teardown(mc_control::fsm::Controller & ctl)
{
  comGenerator_->removeFromLogger(ctl.logger());
}

} /* mc_samples */

EXPORT_SINGLE_STATE("CoMTrajectoryGeneration_Initial", mc_samples::CoMTrajectoryGeneration_Initial)
