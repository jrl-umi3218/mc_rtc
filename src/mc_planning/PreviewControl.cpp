#include <mc_planning/PreviewControl.h>
#include <mc_rtc/constants.h>

namespace mc_planning
{
namespace constants = mc_rtc::constants;

PreviewControl::PreviewControl(const std::string & name)
{
  name_ = "preview_control";
  if(!name.empty())
  {
    name_ += "_" + name;
  }

  reset();
}

void PreviewControl::reset(const Eigen::Vector3d & comState, double zmp)
{
  comState_ = comState;
  zmp_ = zmp;

  f_.resize(0);
  refZmpTraj_.resize(0);
}

void PreviewControl::calcGain(double comHeight, double horizon, double dt)
{
  if(comHeight <= 0 || horizon <= 0 || dt <= 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Input arguments of PreviewControl::calcGain are invalid. comHeight: {}, horizon: {}, dt: {}", comHeight,
        horizon, dt);
  }

  // 1. Set the matrices of linear system
  // clang-format off
    A_ <<
        1.0, dt,  dt * dt / 2.0,
        0.0, 1.0, dt,
        0.0, 0.0, 1.0;

    B_ <<
        dt * dt * dt / 6.0,
        dt * dt / 2.0,
        dt;

    C_ <<
        1.0, 0.0, -comHeight / constants::GRAVITY;
  // clang-format on

  // 2. Calculate P by solving the Discrete-time Algebraic Riccati Equation (DARE)
  // ref. https://scicomp.stackexchange.com/a/35394
  int iterMax = 10000;
  double relNormThre = 1e-8;
  Eigen::Matrix3d A0 = A_;
  Eigen::Matrix3d G0 = (B_ * B_.transpose()) / R_;
  Eigen::Matrix3d H0 = (C_.transpose() * C_) * Q_;
  Eigen::Matrix3d A1, G1, H1;
  for(int i = 0; i < iterMax; i++)
  {
    Eigen::Matrix3d I_GH_inv = (Eigen::Matrix3d::Identity() + G0 * H0).inverse();
    A1 = A0 * I_GH_inv * A0;
    G1 = G0 + A0 * I_GH_inv * G0 * A0.transpose();
    H1 = H0 + A0.transpose() * H0 * I_GH_inv * A0;

    double relNorm = (H1 - H0).norm() / H1.norm();
    if(relNorm < relNormThre)
    {
      break;
    }
    else if(i == iterMax - 1)
    {
      mc_rtc::log::warning("Solution of Riccati equation did not converged: {} > {}", relNorm, relNormThre);
    }

    A0 = A1;
    G0 = G1;
    H0 = H1;
  }
  P_ = H1;

  riccatiError_ = (P_
                   - (A_.transpose() * P_ * A_ + C_.transpose() * C_ * Q_
                      - A_.transpose() * P_ * B_ * B_.transpose() * P_ * A_ / (R_ + B_.transpose() * P_ * B_)))
                      .norm();

  // 3. Calculate the feedback gain (K and f)
  int previewSize = static_cast<int>(std::ceil(horizon / dt));
  double R_BtPB = R_ + B_.dot(P_ * B_);
  // 3.1 Calculate K
  K_ = (B_.transpose() * P_ * A_) / R_BtPB;
  // 3.2 Calculate f
  f_.resize(previewSize);
  Eigen::Matrix3d A_BK = A_ - B_ * K_;
  Eigen::Matrix3d fSub = Eigen::Matrix3d::Identity();
  for(int i = 0; i < previewSize; i++)
  {
    if(i < previewSize - 1)
    {
      f_[i] = (Q_ / R_BtPB) * B_.dot(fSub * C_.transpose());
    }
    else
    {
      // ref. https://github.com/euslisp/jskeus/pull/551
      f_[i] = (1.0 / R_BtPB) * B_.dot(fSub * P_ * C_.transpose());
    }
    fSub = fSub * A_BK.transpose();
  }
}

void PreviewControl::setRefZmpTraj(const Eigen::VectorXd & refZmpTraj)
{
  if(refZmpTraj.size() != previewSize())
  {
    mc_rtc::log::warning("Size of refZmpTraj and previewSize are different: {} != {}", refZmpTraj.size(),
                         previewSize());
    return;
  }
  refZmpTraj_ = refZmpTraj;
}

void PreviewControl::update()
{
  double comJerk = -K_ * comState_ + f_.dot(refZmpTraj_);
  comState_ = A_ * comState_ + B_ * comJerk;
  zmp_ = (C_ * comState_)(0, 0);
}

void PreviewControl::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_com", [this]() { return comState_[0]; });
  logger.addLogEntry(name_ + "_comd", [this]() { return comState_[1]; });
  logger.addLogEntry(name_ + "_comdd", [this]() { return comState_[2]; });
  logger.addLogEntry(name_ + "_zmp", [this]() { return zmp_; });
  logger.addLogEntry(name_ + "_refZmp", [this]() { return refZmpTraj_.size() ? refZmpTraj_[0] : 0.0; });
  logger.addLogEntry(name_ + "_riccatiError", [this]() { return riccatiError_; });
}

void PreviewControl::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_com");
  logger.removeLogEntry(name_ + "_comd");
  logger.removeLogEntry(name_ + "_comdd");
  logger.removeLogEntry(name_ + "_zmp");
  logger.removeLogEntry(name_ + "_refZmp");
  logger.removeLogEntry(name_ + "_riccatiError");
}
} // namespace mc_planning
