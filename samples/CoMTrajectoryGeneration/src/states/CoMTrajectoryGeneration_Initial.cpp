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

  double leftY = leftFootPos_.y()-0.04;
  double rightY = rightFootPos_.y()+0.04;
  double centerY = feetCenterPos_.y();
  const Eigen::Vector3d & currCoM = ctl.robot().com();
  //clang-format off
  m_steps =
  {
    {0.0, currCoM.x(), currCoM.y()},
    {0.2, 0.0, centerY},
    {2,   0.0, centerY},
    {2.2, 0.0, rightY},
    {4,   0.0, rightY},
    {4.2, 0.0, leftY},
    {6,   0.0, leftY},
    {6.2, 0.0, rightY},
    {8,   0.0, rightY},
    {8.2, 0.0, centerY}
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
  mc_rtc::gui::plot::Y("Output CoM", [this]() { return comGenerator_->OutputCOGPosition().y(); }, Color::Magenta),
  mc_rtc::gui::plot::Y("Ideal  CoM", [this]() { return comGenerator_->IdealCOGPosition().y(); }, Color::Red, Style::Dashed),
  mc_rtc::gui::plot::Y("Output ZMP", [this]() { return comGenerator_->OutputZMPPosition().y(); }, Color::Blue),
  mc_rtc::gui::plot::Y("Ideal  ZMP", [this]() { return comGenerator_->IdealZMPPosition().y(); }, Color::Cyan, Style::Dashed));

  ctl.gui()->addElement({"CoMTrajectoryGeneration"},
  mc_rtc::gui::Button("Left",
                      [this,leftY]()
                      {
                        const auto & curr = t_;
                        comGenerator_->changeFutureSteps(iter_,
                                                         {
                                                          Eigen::Vector3d{curr + previewTime_ + 0.8, 0, leftY},
                                                         });
                        updateSteps();
                      }),
  mc_rtc::gui::Button("Right",
                      [this,rightY]()
                      {
                        const auto & curr = t_;
                        comGenerator_->changeFutureSteps(iter_,
                                                         {
                                                          Eigen::Vector3d{curr + previewTime_ + 0.8, 0, rightY},
                                                         });
                        updateSteps();
                      }),
  mc_rtc::gui::Button("Center",
                      [this,centerY]()
                      {
                        const auto & curr = t_;
                        comGenerator_->changeFutureSteps(iter_,
                                                         {
                                                          Eigen::Vector3d{curr + previewTime_ + 0.8, 0, centerY},
                                                         });
                        updateSteps();
                        }),
  mc_rtc::gui::Button("Left-Right-Left",
                      [this,centerY,leftY,rightY]()
                      {
                        const auto & curr = t_;
                        comGenerator_->changeFutureSteps(iter_,
                                                         {
                                                          Eigen::Vector3d{curr + previewTime_ + 0.2, 0, leftY},
                                                          Eigen::Vector3d{curr + previewTime_ + 2, 0, leftY},
                                                          Eigen::Vector3d{curr + previewTime_ + 2.2, 0, rightY},
                                                          Eigen::Vector3d{curr + previewTime_ + 4, 0, rightY},
                                                          Eigen::Vector3d{curr + previewTime_ + 4.2, 0, centerY},
                                                         });
                        updateSteps();
                        }),
  mc_rtc::gui::Button("Left-Right-Left (x10)",
                      [this,centerY,leftY,rightY]()
                      {
                        const auto & curr = t_;
                        std::vector<Eigen::Vector3d> futureSteps;
                        auto startT = curr;
                        for (int i = 0; i < 10; ++i)
                        {
                          futureSteps.push_back(Eigen::Vector3d{startT + previewTime_ + 0.2, 0, leftY});
                          futureSteps.push_back(Eigen::Vector3d{startT + previewTime_ + 2, 0, leftY});
                          futureSteps.push_back(Eigen::Vector3d{startT + previewTime_ + 2.2, 0, rightY});
                          futureSteps.push_back(Eigen::Vector3d{startT + previewTime_ + 4, 0, rightY});
                          startT += 4;
                        }
                        futureSteps.push_back(Eigen::Vector3d{startT + previewTime_ + 0.2, 0, centerY});
                        comGenerator_->changeFutureSteps(iter_, futureSteps);
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
  mc_rtc::log::info("Current time: {}", t_);
  mc_rtc::log::info(
      "Desired steps:\nTime\tCoM X\tCoM Y\n{}",
      mc_rtc::io::to_string(comGenerator_->steps(), [](const Eigen::Vector3d & v) -> Eigen::RowVector3d { return v; }, "\n"));
}

bool CoMTrajectoryGeneration_Initial::run(mc_control::fsm::Controller & ctl)
{
  /* Generate trajectory starting at iter_ time (start of the past preview window)
   * The trajectory result is computed for the current time (iter_+previewSize_)
   */
  auto generateTrajectory = [this]()
  {
    comGenerator_->generate(iter_);
  };
  generationTime_ = mc_rtc::measure_ms::execution(generateTrajectory).count();

  // Provide targets to the stabilizer (running in another state)
  if(ctl.datastore().has("StabilizerStandingTrackCoM::target"))
  {
    ctl.datastore().call("StabilizerStandingTrackCoM::target", comGenerator_->OutputCOGPosition(), comGenerator_->OutputCOGVelocity(), comGenerator_->OutputCOGAcceleration(), comGenerator_->OutputZMPPosition());
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
