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
  preview_ = mc_planning::CenteredPreviewWindow(previewTime_, ctl.timeStep);
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
  steps_.reset(
  {
    {0.0, {currCoM.x(), currCoM.y()}},
    {0.2, {0.0, centerY}},
    {2,   {0.0, centerY}},
    {2.2, {0.0, rightY}},
    {4,   {0.0, rightY}},
    {4.2, {0.0, leftY}},
    {6,   {0.0, leftY}},
    {6.2, {0.0, rightY}},
    {8,   {0.0, rightY}},
    {8.2, {0.0, centerY}}
  });
  steps_.initialize();
  //clang-format on

  comGenerator_ = std::make_shared<mc_planning::generator>(preview_, ctl.robot().mass(),
                                                           waist_height);
  comGenerator_->addToLogger(ctl.logger());
  comGenerator_->steps(steps_);
  updateSteps();

  using Color = mc_rtc::gui::Color;
  using Style = mc_rtc::gui::plot::Style;
  ctl.gui()->addPlot(
  "Trajectory (Y)",
  mc_rtc::gui::plot::X("t", [this]() { return t_; }),
  mc_rtc::gui::plot::Y("Output CoM", [this]() { return comGenerator_->OutputCOGPosition().y(); }, Color::Magenta),
  mc_rtc::gui::plot::Y("Ideal  CoM", [this]() { return comGenerator_->IdealCOGPosition().y(); }, Color::Red, Style::Dashed),
  mc_rtc::gui::plot::Y("Output ZMP", [this]() { return comGenerator_->OutputZMPPosition().y(); }, Color::Purple),
  mc_rtc::gui::plot::Y("Ideal  ZMP", [this]() { return comGenerator_->IdealZMPPosition().y(); }, Color::Blue, Style::Dashed));

  ctl.gui()->addElement({"CoMTrajectoryGeneration"},
  mc_rtc::gui::NumberInput("Delay",
                           [this]()
                           {
                            return delay_;
                           },
                           [this](double delay)
                           {
                            delay_ = delay;
                           }),
  mc_rtc::gui::NumberInput("Transition time",
                           [this]()
                           {
                            return transition_;
                           },
                           [this](double transition)
                           {
                            transition_ = transition;
                           }),
  mc_rtc::gui::Button("Left",
                      [this,leftY]()
                      {
                        auto curr = t_ + previewTime_ + delay_;
                        comGenerator_->changeFutureSteps(
                            curr,
                            {
                              {curr + transition_, {0, leftY}}
                            });
                        updateSteps();
                      }),
  mc_rtc::gui::Button("Right",
                      [this,rightY]()
                      {
                        auto curr= t_ + previewTime_ + delay_;
                        comGenerator_->changeFutureSteps(
                            curr,
                            { {curr + transition_, {0, rightY}} });
                        updateSteps();
                      }),
  mc_rtc::gui::Button("Center",
                      [this,centerY]()
                      {
                        auto curr= t_ + previewTime_ + delay_;
                        comGenerator_->changeFutureSteps(
                            curr,
                            { {curr + transition_, {0, centerY}} });
                        updateSteps();
                        }),
  mc_rtc::gui::Button("Left-Right-Left",
                      [this,centerY,leftY,rightY]()
                      {
                        auto curr= t_ + previewTime_ + delay_;
                        std::vector<mc_planning::TimedStep<Eigen::Vector2d>> futureSteps;
                        auto t = curr + transition_;
                        futureSteps.push_back({t, {0, leftY}});
                        t += 2;
                        futureSteps.push_back({t,   {0, leftY}});
                        t += transition_;
                        futureSteps.push_back({t, {0, rightY}});
                        t += 2;
                        futureSteps.push_back({t,   {0, rightY}});
                        t += transition_;
                        futureSteps.push_back({t, {0, centerY}});
                        comGenerator_->changeFutureSteps(curr, futureSteps);
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
  mc_rtc::log::info("Time start: {}", t_);
  mc_rtc::log::info("Time curr : {}", t_+previewTime_);
  mc_rtc::log::info(
      "Desired steps:\nTime\tCoM X\tCoM Y\n{}",
      mc_rtc::io::to_string(comGenerator_->steps().steps(), "\n"));
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
  t_ += ctl.timeStep;
  output("OK");
  return false;
}

void CoMTrajectoryGeneration_Initial::teardown(mc_control::fsm::Controller & ctl)
{
  comGenerator_->removeFromLogger(ctl.logger());
}

} /* mc_samples */

EXPORT_SINGLE_STATE("CoMTrajectoryGeneration_Initial", mc_samples::CoMTrajectoryGeneration_Initial)
