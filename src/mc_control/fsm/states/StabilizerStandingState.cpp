#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/StabilizerStandingState.h>
#include <mc_rtc/constants.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

namespace mc_control
{
namespace fsm
{

namespace constants = mc_rtc::constants;
using ContactState = mc_tasks::lipm_stabilizer::ContactState;

void StabilizerStandingState::configure(const mc_rtc::Configuration & config)
{
  if(config.has("completion"))
  {
    hasCompletion_ = !config("completion").empty();
    config("completion")("dcmEval", dcmThreshold_);
  }
  config("optionalGUI", optionalGUI_);

  config_.load(config);
}

void StabilizerStandingState::start(Controller & ctl)
{
  if(!config_.has("StabilizerConfig"))
  {
    config_.add("StabilizerConfig");
  }
  config_("StabilizerConfig").add("type", "lipm_stabilizer");

  config_("stiffness", K_);
  D_ = config_("damping", 2 * std::sqrt(K_));

  // create stabilizer task from config
  stabilizerTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::StabilizerTask>(
      ctl.solver(), config_("StabilizerConfig"));
  robot_ = stabilizerTask_->robot().name();
  auto & robot = ctl.robot(robot_);
  anchorFrameFunction_ = config_("anchorFrameFunction", "KinematicAnchorFrame::" + robot_);
  ctl.solver().addTask(stabilizerTask_);

  // Initialize stabilizer targets. Defaults to current CoM/CoP
  config_("comHeight", stabilizerTask_->config().comHeight);
  // Reset linear inverted pendulum model, used here to compute stabilizer references
  double lambda = constants::GRAVITY / stabilizerTask_->config().comHeight;
  pendulum_.reset(lambda, robot.com(), robot.comVelocity(), robot.comAcceleration());
  if(config_.has("above"))
  {
    const std::string above = config_("above");
    if(above == "LeftAnkle")
    {
      targetCoP(stabilizerTask_->contactAnklePose(ContactState::Left).translation());
    }
    else if(above == "RightAnkle")
    {
      targetCoP(stabilizerTask_->contactAnklePose(ContactState::Right).translation());
    }
    else if(above == "CenterAnkles")
    {
      targetCoP(sva::interpolate(stabilizerTask_->contactAnklePose(ContactState::Left),
                                 stabilizerTask_->contactAnklePose(ContactState::Right), 0.5)
                    .translation());
    }
    else if(above == "LeftSurface")
    {
      targetCoP(robot.surfacePose(stabilizerTask_->footSurface(ContactState::Left)).translation());
    }
    else if(above == "RightSurface")
    {
      targetCoP(robot.surfacePose(stabilizerTask_->footSurface(ContactState::Right)).translation());
    }
    else if(above == "CenterSurfaces")
    {
      targetCoP(sva::interpolate(ctl.robot().surfacePose(stabilizerTask_->footSurface(ContactState::Left)),
                                 ctl.robot().surfacePose(stabilizerTask_->footSurface(ContactState::Right)), 0.5)
                    .translation());
    }
    else if(robot.hasSurface(above))
    {
      targetCoP(robot.surfacePose(above).translation());
    }
    else
    {
      mc_rtc::log::error_and_throw(
          "[StabilizerStandingState] Requested standing above {} but this is neither one of the state target "
          "(LeftAnkle, RightAnkle, CenterAnkles, LeftSurface, RightSurface, CenterSurfaces), nor a valid robot surface "
          "name",
          above);
    }
  }
  else if(config_.has("com"))
  {
    targetCoM(config_("com"));
  }
  else
  {
    targetCoM(robot.com());
  }

  // Fixme: the stabilizer needs the observed state immediatly
  if(ctl.datastore().has(anchorFrameFunction_))
  {
    mc_rtc::log::warning("[{}] a datastore callback for \"{}\" already exist on the datastore, using it instead",
                         name(), anchorFrameFunction_);
    ownsAnchorFrameCallback_ = false;
  }
  else
  {
    ctl.datastore().make_call(anchorFrameFunction_,
                              [this](const mc_rbdyn::Robot & robot) { return stabilizerTask_->anchorFrame(robot); });
    ownsAnchorFrameCallback_ = true;
  }

  if(optionalGUI_ && stabilizerTask_->inDoubleSupport())
  {
    ctl.gui()->addElement(
        {"FSM", name(), "Move"}, mc_rtc::gui::ElementsStacking::Horizontal,
        mc_rtc::gui::Button(
            "Left foot", [this]() { targetCoP(stabilizerTask_->contactAnklePose(ContactState::Left).translation()); }),
        mc_rtc::gui::Button("Center",
                            [this]() {
                              targetCoP(sva::interpolate(stabilizerTask_->contactAnklePose(ContactState::Left),
                                                         stabilizerTask_->contactAnklePose(ContactState::Right), 0.5)
                                            .translation());
                            }),
        mc_rtc::gui::Button("Right foot", [this]() {
          targetCoP(stabilizerTask_->contactAnklePose(ContactState::Right).translation());
        }));
    ctl.gui()->addElement({"FSM", name(), "Move"},
                          mc_rtc::gui::ArrayInput(
                              "CoM Target", [this]() -> const Eigen::Vector3d & { return comTarget_; },
                              [this](const Eigen::Vector3d & com) { targetCoM(com); }),
                          mc_rtc::gui::ArrayInput(
                              "Move CoM", []() -> Eigen::Vector3d { return Eigen::Vector3d::Zero(); },
                              [this](const Eigen::Vector3d & com) { targetCoM(comTarget_ + com); }));
  }

  ctl.gui()->addElement({"FSM", name(), "Gains"},
                        mc_rtc::gui::NumberInput(
                            "CoM stiffness", [this]() { return K_; }, [this](const double & s) { K_ = s; }),
                        mc_rtc::gui::NumberInput(
                            "CoM damping", [this]() { return D_; }, [this](const double & d) { D_ = d; }),
                        mc_rtc::gui::NumberInput(
                            "CoM stiffness & damping", [this]() { return K_; },
                            [this](const double & g) {
                              K_ = g;
                              D_ = 2 * std::sqrt(K_);
                            }));

#define LOG_MEMBER(NAME, MEMBER) MC_RTC_LOG_HELPER(name() + NAME, MEMBER)
  auto & logger = ctl.logger();
  LOG_MEMBER("_stiffness", K_);
  LOG_MEMBER("_damping", D_);
  LOG_MEMBER("_targetCoM", comTarget_);
  LOG_MEMBER("_targetCoP", copTarget_);
#undef LOG_MEMBER

  // Provide accessor callbacks on the datastore
  ctl.datastore().make_call("StabilizerStandingState::getCoMTarget",
                            [this]() -> const Eigen::Vector3d & { return comTarget_; });
  ctl.datastore().make_call("StabilizerStandingState::setCoMTarget",
                            [this](const Eigen::Vector3d & com) { this->targetCoM(com); });
  ctl.datastore().make_call("StabilizerStandingState::setStiffness", [this](double K) { this->K_ = K; });
  ctl.datastore().make_call("StabilizerStandingState::setDamping", [this](double D) { this->D_ = D; });
  ctl.datastore().make_call("StabilizerStandingState::getStiffness", [this]() { return K_; });
  ctl.datastore().make_call("StabilizerStandingState::getDamping", [this]() { return D_; });
  ctl.datastore().make_call(
      "StabilizerStandingState::getConfiguration",
      [this]() -> mc_rbdyn::lipm_stabilizer::StabilizerConfiguration { return stabilizerTask_->config(); });
  ctl.datastore().make_call(
      "StabilizerStandingState::setConfiguration",
      [this](const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & conf) { stabilizerTask_->configure(conf); });
  ctl.datastore().make_call("StabilizerStandingState::setPelvisWeight",
                            [this](double w) { stabilizerTask_->pelvisWeight(w); });
  ctl.datastore().make_call("StabilizerStandingState::setPelvisStiffness",
                            [this](double s) { stabilizerTask_->pelvisStiffness(s); });
  ctl.datastore().make_call("StabilizerStandingState::setTorsoWeight",
                            [this](double w) { stabilizerTask_->torsoWeight(w); });
  ctl.datastore().make_call("StabilizerStandingState::setTorsoStiffness",
                            [this](double s) { stabilizerTask_->torsoStiffness(s); });
  ctl.datastore().make_call("StabilizerStandingState::setCoMWeight",
                            [this](double w) { stabilizerTask_->comWeight(w); });
  ctl.datastore().make_call("StabilizerStandingState::setCoMStiffness",
                            [this](const Eigen::Vector3d & s) { stabilizerTask_->comStiffness(s); });
  ctl.datastore().make_call("StabilizerStandingState::setExternalWrenches",
                            [this](const std::vector<std::string> & surfaceNames,
                                   const std::vector<sva::ForceVecd> & targetWrenches,
                                   const std::vector<sva::MotionVecd> & gains) {
                              stabilizerTask_->setExternalWrenches(surfaceNames, targetWrenches, gains);
                            });
  ctl.datastore().make_call("StabilizerStandingState::getCoPAdmittance",
                            [this]() { return stabilizerTask_->config().copAdmittance; });
  ctl.datastore().make_call("StabilizerStandingState::setCoPAdmittance", [this](const Eigen::Vector2d & copAdmittance) {
    stabilizerTask_->copAdmittance(copAdmittance);
  });
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
    copHeight = (stabilizerTask_->contactAnklePose(ContactState::Left).translation().z()
                 + stabilizerTask_->contactAnklePose(ContactState::Right).translation().z())
                / 2;
  }
  else if(stabilizerTask_->inContact(ContactState::Left))
  {
    copHeight = stabilizerTask_->contactAnklePose(ContactState::Left).translation().z();
  }
  else
  {
    copHeight = stabilizerTask_->contactAnklePose(ContactState::Right).translation().z();
  }

  comTarget_ = com;
  copTarget_ = Eigen::Vector3d{comTarget_.x(), comTarget_.y(), copHeight};
}

bool StabilizerStandingState::run(Controller & ctl)
{
  const Eigen::Vector3d & com_ = pendulum_.com();
  const Eigen::Vector3d & comd_ = pendulum_.comd();

  Eigen::Vector3d comdd = K_ * (comTarget_ - com_) - D_ * comd_;
  Eigen::Vector3d n = constants::vertical;
  double lambda = n.dot(comdd + constants::gravity) / n.dot(com_ - copTarget_);
  Eigen::Vector3d zmp = com_ - (constants::gravity + comdd) / lambda;

  pendulum_.integrateIPM(zmp, lambda, ctl.timeStep);

  // Update stabilizer target
  stabilizerTask_->target(pendulum_.com(), pendulum_.comd(), pendulum_.comdd(), pendulum_.zmp());

  if(!hasCompletion_)
  {
    output("OK");
    return true;
  }
  const auto & dcm = stabilizerTask_->measuredDCM();
  if((((dcm - comTarget_).cwiseAbs() - dcmThreshold_).array() < 0.).all())
  {
    output("OK");
    return true;
  }
  return false;
}

void StabilizerStandingState::teardown(Controller & ctl)
{
  ctl.solver().removeTask(stabilizerTask_);
  ctl.gui()->removeCategory({"FSM", name()});
  ctl.logger().removeLogEntries(this);

  ctl.datastore().remove("StabilizerStandingState::getCoMTarget");
  ctl.datastore().remove("StabilizerStandingState::setCoMTarget");
  ctl.datastore().remove("StabilizerStandingState::getStiffness");
  ctl.datastore().remove("StabilizerStandingState::setStiffness");
  ctl.datastore().remove("StabilizerStandingState::getDamping");
  ctl.datastore().remove("StabilizerStandingState::setDamping");
  ctl.datastore().remove("StabilizerStandingState::getConfiguration");
  ctl.datastore().remove("StabilizerStandingState::setConfiguration");
  ctl.datastore().remove("StabilizerStandingState::setPelvisWeight");
  ctl.datastore().remove("StabilizerStandingState::setPelvisStiffness");
  ctl.datastore().remove("StabilizerStandingState::setTorsoWeight");
  ctl.datastore().remove("StabilizerStandingState::setTorsoStiffness");
  ctl.datastore().remove("StabilizerStandingState::setCoMWeight");
  ctl.datastore().remove("StabilizerStandingState::setCoMStiffness");
  ctl.datastore().remove("StabilizerStandingState::setExternalWrenches");
  ctl.datastore().remove("StabilizerStandingState::getCoPAdmittance");
  ctl.datastore().remove("StabilizerStandingState::setCoPAdmittance");
  if(ownsAnchorFrameCallback_)
  {
    ctl.datastore().remove(anchorFrameFunction_);
  }
}

} // namespace fsm
} // namespace mc_control
EXPORT_SINGLE_STATE("StabilizerStandingState", mc_control::fsm::StabilizerStandingState)
