#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/StabilizerStandingState.h>
#include <mc_rbdyn/World.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_control
{
namespace fsm
{

namespace world = mc_rbdyn::world;
using ContactState = mc_tasks::lipm_stabilizer::ContactState;

StabilizerStandingState::StabilizerStandingState() {}

void StabilizerStandingState::configure(const mc_rtc::Configuration & config)
{
  config("target", leftFootRatio_);
  config("stiffness", K_);
  if(config.has("damping"))
  {
    D_ = config("damping");
  }
  else
  {
    D_ = 2 * std::sqrt(K_);
  }
  config("contactState", contactState_);
  if(config.has("completion"))
  {
    hasCompletion_ = true;
    config("completion")("dcmEval", dcmThreshold_);
  }
  config_.load(config);
}

void StabilizerStandingState::start(Controller & ctl)
{
  // create stabilizer task from config
  if(!config_.has("StabilizerTask"))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "[StabilizerStandingState] expects to have a StabilizerTask configuration "
                                            "entry with the LIPMStabilizerTask configuration");
  }
  stabilizerTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::StabilizerTask>(
      ctl.solver(), config_("StabilizerTask"));
  ctl.solver().addTask(stabilizerTask_);

  pendulum_.reset(ctl.robot().com(), ctl.robot().comVelocity(), ctl.robot().comAcceleration());

  stabilizerTask_->contactState(contactState_);
  target(ctl, leftFootRatio_);

  if(contactState_ == ContactState::DoubleSupport)
  {
    ctl.gui()->addElement({"Standing"}, mc_rtc::gui::Button("Left foot", [this, &ctl]() { target(ctl, 0); }),
                          mc_rtc::gui::Button("Center", [this, &ctl]() { target(ctl, 0.5); }),
                          mc_rtc::gui::Button("Right foot", [this, &ctl]() { target(ctl, 1); }));
  }
}

void StabilizerStandingState::target(const Controller & ctl, double leftFootRatio)
{
  leftFootRatio_ = leftFootRatio;

  const auto & lf = stabilizerTask_->config().leftFootSurface;
  const auto & rf = stabilizerTask_->config().rightFootSurface;
  sva::PTransformd X_0_lfr = sva::interpolate(ctl.robot().surfacePose(lf), ctl.robot().surfacePose(rf), leftFootRatio_);
  copTarget_ = X_0_lfr.translation();
  comTarget_ = copTarget_ + Eigen::Vector3d{0., 0., stabilizerTask_->config().comHeight};
}

bool StabilizerStandingState::run(Controller & ctl)
{
  const Eigen::Vector3d & com_ = pendulum_.com();
  const Eigen::Vector3d & comd_ = pendulum_.comd();

  Eigen::Vector3d comdd = K_ * (comTarget_ - com_) - D_ * comd_;
  // FIXME Hardcoded, should be the ideal contact normal
  // To be fixed after refactoring of the Contact task
  Eigen::Vector3d n = Eigen::Vector3d{0., 0., 1.};
  double lambda = n.dot(comdd - world::gravity) / n.dot(com_ - copTarget_);
  Eigen::Vector3d zmp = com_ + (world::gravity - comdd) / lambda;

  pendulum_.integrateIPM(zmp, lambda, ctl.timeStep);

  stabilizerTask_->target(pendulum_.com(), pendulum_.comd(), pendulum_.comdd(), pendulum_.zmp());

  output("OK");
  if(!hasCompletion_)
  {
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
  ctl.gui()->removeCategory({"Standing"});
}

} // namespace fsm
} // namespace mc_control
EXPORT_SINGLE_STATE("StabilizerStandingState", mc_control::fsm::StabilizerStandingState)
