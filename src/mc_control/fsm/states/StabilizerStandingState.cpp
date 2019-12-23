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

StabilizerStandingState::StabilizerStandingState()
{
  // XXX, it would be nicer if calling mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::StabilizerTask>() would
  // not require the configuration to have a type element as it is redundant in that case
  config_.add("StabilizerConfig");
  config_("StabilizerConfig").add("type", "lipm_stabilizer");
}

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
  if(config.has("completion"))
  {
    if(config("completion").empty())
    {
      hasCompletion_ = false;
    }
    else
    {
      hasCompletion_ = true;
      config("completion")("dcmEval", dcmThreshold_);
    }
  }
  config("contactState", contactState_);
  config_.load(config);
}

void StabilizerStandingState::start(Controller & ctl)
{

  // create stabilizer task from config
  stabilizerTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::StabilizerTask>(
      ctl.solver(), config_("StabilizerConfig", mc_rtc::Configuration{}));
  ctl.solver().addTask(stabilizerTask_);

  pendulum_.reset(ctl.robot().com(), ctl.robot().comVelocity(), ctl.robot().comAcceleration());

  stabilizerTask_->contactState(contactState_);
  target(leftFootRatio_);

  if(contactState_ == ContactState::DoubleSupport)
  {
    ctl.gui()->addElement({"Standing"}, mc_rtc::gui::Button("Left foot", [this, &ctl]() { target(0); }),
                          mc_rtc::gui::Button("Center", [this, &ctl]() { target(0.5); }),
                          mc_rtc::gui::Button("Right foot", [this, &ctl]() { target(1); }));
  }

  output("OK");
}

void StabilizerStandingState::target(double leftFootRatio)
{
  leftFootRatio_ = leftFootRatio;

  if(contactState_ == ContactState::DoubleSupport)
  {
    const auto & lf = stabilizerTask_->leftContactAnklePose();
    const auto & rf = stabilizerTask_->rightContactAnklePose();
    sva::PTransformd X_0_lfr = sva::interpolate(lf, rf, leftFootRatio_);
    copTarget_ = X_0_lfr.translation();
  }
  else if(contactState_ == ContactState::Left)
  {
    copTarget_ = stabilizerTask_->leftContactAnklePose().translation();
  }
  else
  {
    copTarget_ = stabilizerTask_->rightContactAnklePose().translation();
  }

  comTarget_ = copTarget_ + Eigen::Vector3d{0., 0., stabilizerTask_->config().comHeight};
}

bool StabilizerStandingState::run(Controller & ctl)
{
  const Eigen::Vector3d & com_ = pendulum_.com();
  const Eigen::Vector3d & comd_ = pendulum_.comd();

  Eigen::Vector3d comdd = K_ * (comTarget_ - com_) - D_ * comd_;
  Eigen::Vector3d n = Eigen::Vector3d{0., 0., 1.};
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
    return true;
  }
  if(std::fabs(stabilizerTask_->measuredDCM().x() - comTarget_.x()) < dcmThreshold_.x()
     && std::fabs(stabilizerTask_->measuredDCM().y() - comTarget_.y()) < dcmThreshold_.y()
     && std::fabs(stabilizerTask_->measuredDCM().z() - comTarget_.z()) < dcmThreshold_.z())
  {
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
