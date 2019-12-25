#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/StabilizerStandingState.h>
#include <mc_rbdyn/World.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

namespace mc_control
{
namespace fsm
{

namespace world = mc_rbdyn::world;
using ContactState = mc_tasks::lipm_stabilizer::ContactState;

StabilizerStandingState::StabilizerStandingState()
{
  // XXX, it would be nicer if calling mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::StabilizerTask>() would
  // not require the configuration to have a type element as it is redundant in that case
  config_.add("StabilizerConfig");
  config_("StabilizerConfig").add("type", "lipm_stabilizer");
}

void StabilizerStandingState::configure(const mc_rtc::Configuration & config)
{
  if(config.has("completion"))
  {
    hasCompletion_ = !config("completion").empty();
    config("completion")("dcmEval", dcmThreshold_);
  }

  config_.load(config);
}

void StabilizerStandingState::start(Controller & ctl)
{
  config_("stiffness", K_);
  D_ = config_("damping", 2 * std::sqrt(K_));

  // create stabilizer task from config
  stabilizerTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::StabilizerTask>(
      ctl.solver(), config_("StabilizerConfig", mc_rtc::Configuration{}));
  ctl.solver().addTask(stabilizerTask_);

  // Reset linear inverted pendulum model, used here to compute stabilizer
  // references
  pendulum_.reset(ctl.robot().com(), ctl.robot().comVelocity(), ctl.robot().comAcceleration());

  // Initialize stabilizer targets. Defaults to current CoM/CoP
  config_("comHeight", stabilizerTask_->config().comHeight);
  if(config_.has("above"))
  {
    const std::string above = config_("above");
    if(above == "LeftAnkle")
    {
      targetCoP(stabilizerTask_->leftContactAnklePose().translation());
    }
    else if(above == "RightAnkle")
    {
      targetCoP(stabilizerTask_->rightContactAnklePose().translation());
    }
    else if(above == "Center")
    {
      targetCoP(sva::interpolate(stabilizerTask_->leftContactAnklePose(), stabilizerTask_->rightContactAnklePose(), 0.5)
                    .translation());
    }
    else if(ctl.realRobot().hasSurface(above))
    {
      targetCoP(ctl.realRobot().surfacePose(above).translation());
    }
    else
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "[StabilizerStandingState] Requested standing above "
                                                  << above
                                                  << " but this is neither one of the state target (LeftAnkle, "
                                                     "RightAnkle, Center), nor a valid robot surface");
    }
  }
  else if(config_.has("com"))
  {
    targetCoM(config_("com"));
  }
  else
  {
    targetCoM(ctl.realRobot().com());
  }

  if(stabilizerTask_->inDoubleSupport())
  {
    ctl.gui()->addElement(
        {"FSM", "Standing", "Move"}, mc_rtc::gui::ElementsStacking::Horizontal,
        mc_rtc::gui::Button("Left foot",
                            [this]() { targetCoP(stabilizerTask_->leftContactAnklePose().translation()); }),
        mc_rtc::gui::Button("Center",
                            [this]() {
                              targetCoP(sva::interpolate(stabilizerTask_->leftContactAnklePose(),
                                                         stabilizerTask_->rightContactAnklePose(), 0.5)
                                            .translation());
                            }),
        mc_rtc::gui::Button("Right foot",
                            [this]() { targetCoP(stabilizerTask_->rightContactAnklePose().translation()); }));
    ctl.gui()->addElement(
        {"FSM", "Standing", "Move"},
        mc_rtc::gui::ArrayInput("CoM Target", [this]() -> const Eigen::Vector3d & { return comTarget_; },
                                [this](const Eigen::Vector3d & com) { targetCoM(com); }),
        mc_rtc::gui::ArrayInput("Move CoM", []() -> Eigen::Vector3d { return Eigen::Vector3d::Zero(); },
                                [this](const Eigen::Vector3d & com) { targetCoM(comTarget_ + com); }));
  }

  ctl.gui()->addElement(
      {"FSM", "Standing", "Gains"},
      mc_rtc::gui::NumberInput("CoM stiffness", [this]() { return K_; }, [this](const double & s) { K_ = s; }),
      mc_rtc::gui::NumberInput("CoM damping", [this]() { return D_; }, [this](const double & d) { D_ = d; }),
      mc_rtc::gui::NumberInput("CoM stiffness & damping", [this]() { return K_; },
                               [this](const double & g) {
                                 K_ = g;
                                 D_ = 2 * std::sqrt(K_);
                               }));

  ctl.logger().addLogEntry(name() + "_stiffness", [this]() { return K_; });
  ctl.logger().addLogEntry(name() + "_damping", [this]() { return D_; });
  ctl.logger().addLogEntry(name() + "_targetCoM", [this]() -> const Eigen::Vector3d & { return comTarget_; });
  ctl.logger().addLogEntry(name() + "_targetCoP", [this]() -> const Eigen::Vector3d & { return copTarget_; });
}

void StabilizerStandingState::targetCoP(const Eigen::Vector3d & cop)
{
  comTarget_ = cop + Eigen::Vector3d{0., 0., stabilizerTask_->config().comHeight};
  copTarget_ = cop;
}

void StabilizerStandingState::targetCoM(const Eigen::Vector3d & com)
{
  double copHeight = 0;
  if(stabilizerTask_->inDoubleSupport())
  {
    copHeight = (stabilizerTask_->leftContactAnklePose().translation().z()
                 + stabilizerTask_->rightContactAnklePose().translation().z())
                / 2;
  }
  else if(stabilizerTask_->inContact(ContactState::Left))
  {
    copHeight = stabilizerTask_->leftContactAnklePose().translation().z();
  }
  else
  {
    copHeight = stabilizerTask_->rightContactAnklePose().translation().z();
  }

  // Initialize stabilizer at current CoM position
  comTarget_ = com;
  copTarget_ = Eigen::Vector3d{comTarget_.x(), comTarget_.y(), copHeight};
}

bool StabilizerStandingState::run(Controller & ctl)
{
  const Eigen::Vector3d & com_ = pendulum_.com();
  const Eigen::Vector3d & comd_ = pendulum_.comd();

  Eigen::Vector3d comdd = K_ * (comTarget_ - com_) - D_ * comd_;
  Eigen::Vector3d n = mc_rbdyn::world::vertical;
  double lambda = n.dot(comdd - world::gravity) / n.dot(com_ - copTarget_);
  Eigen::Vector3d zmp = com_ + (world::gravity - comdd) / lambda;

  pendulum_.integrateIPM(zmp, lambda, ctl.timeStep);

  // Update stabilizer target
  stabilizerTask_->target(pendulum_.com(), pendulum_.comd(), pendulum_.comdd(), pendulum_.zmp());
  // Update anchor frame for the KinematicInertial observer
  ctl.anchorFrame(stabilizerTask_->anchorFrame());
  ctl.anchorFrameReal(stabilizerTask_->anchorFrameReal());

  if(!hasCompletion_)
  {
    output("OK");
    return true;
  }
  if(std::fabs(stabilizerTask_->measuredDCM().x() - comTarget_.x()) < dcmThreshold_.x()
     && std::fabs(stabilizerTask_->measuredDCM().y() - comTarget_.y()) < dcmThreshold_.y()
     && std::fabs(stabilizerTask_->measuredDCM().z() - comTarget_.z()) < dcmThreshold_.z())
  {
    output("OK");
    return true;
  }
  return false;
}

void StabilizerStandingState::teardown(Controller & ctl)
{
  ctl.solver().removeTask(stabilizerTask_);
  ctl.gui()->removeCategory({"FSM", "Standing"});
  ctl.logger().removeLogEntry(name() + "_stiffness");
  ctl.logger().removeLogEntry(name() + "_damping");
  ctl.logger().removeLogEntry(name() + "_targetCoM");
  ctl.logger().removeLogEntry(name() + "_targetCoP");
}

} // namespace fsm
} // namespace mc_control
EXPORT_SINGLE_STATE("StabilizerStandingState", mc_control::fsm::StabilizerStandingState)
