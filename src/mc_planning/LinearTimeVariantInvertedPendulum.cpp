#include <mc_planning/LinearTimeVariantInvertedPendulum.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>

namespace cst = mc_rtc::constants;

namespace mc_planning
{

namespace linear_control_system
{

LinearTimeVariantInvertedPendulum::LinearTimeVariantInvertedPendulum(const CenteredPreviewWindow & window,
                                                                     unsigned weight_resolution,
                                                                     double minHeight,
                                                                     double maxHeight)
: window_(window)
{
  if(weight_resolution == 0)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[LinearTimeVariantInvertedPendulum::Initialize] Invalid "
                                                        "weight resolution {} (must be >0, recommended value 20000) ",
                                                        weight_resolution);
  }

  /** Create lookup tables for sqrt(w2), cosh(w*dt), sinh(w*dt) */
  auto omega2 = [](double h) { return mc_rtc::constants::GRAVITY / h; };
  auto omega = [](double w2) { return std::sqrt(w2); };
  auto chk = [this](double w2) { return cosh(wTable_(w2) * window_.dt()); };
  auto shk = [this](double w2) { return sinh(wTable_(w2) * window_.dt()); };
  double minOmega2 = omega2(maxHeight);
  double maxOmega2 = omega2(minHeight);
  wTable_.create(weight_resolution, minOmega2, maxOmega2, omega, "omega^2 -> sqrt(w2)");
  chkTable_.create(weight_resolution, minOmega2, maxOmega2, chk, "omega^2 -> cosh(sqrt(omega^2 * dt)");
  shkTable_.create(weight_resolution, minOmega2, maxOmega2, shk, "omega^2 -> sinh(sqrt(omega^2 * dt)");

  unsigned previewSize = window_.size();
  m_A.resize(previewSize, Matrix22::Identity());
  m_An.resize(previewSize, Matrix22::Identity());
  m_B.resize(previewSize, Vector2::Zero());
  m_Bn.resize(previewSize, Vector2::Zero());

  m_X.resize(previewSize, Vector2::Zero());
  m_p_ref.setZero(previewSize);
  m_w2.setZero(previewSize);
  m_w.setZero(previewSize);
}

void LinearTimeVariantInvertedPendulum::initMatrices(double waist_height)
{
  const double w2 = cst::GRAVITY / waist_height;

  // Initialize A matrix corresponding to the given pendulum height
  const double w = wTable_(w2);
  const double ch = chkTable_(w2);
  const double sh = shkTable_(w2);

  auto init_m22 = [&]() {
    Eigen::Matrix2d A;
    A << ch, sh / w, w * sh, ch;

    for(unsigned i = 0; i < window_.size(); i++)
    {
      m_A.push_front(A);
    }
  };

  auto init_v2 = [&]() {
    Vector2 v;
    v << 1 - ch, -w * sh;

    for(unsigned i = 0; i < window_.size(); i++)
    {
      m_B.push_front(v);
    }
  };

  // Initialize the whole preview window at constant height
  init_m22();
  init_v2();
}

void LinearTimeVariantInvertedPendulum::update()
{
  // set newest matrices
  {
    double w2 = m_w2[window_.last()];
    const double w = wTable_(w2);
    const double ch = chkTable_(w2);
    const double sh = shkTable_(w2);
    m_w[window_.last()] = w;
    Matrix22 Anew;
    Anew << ch, sh / w, w * sh, ch;
    Vector2 Bnew;
    Bnew << 1 - ch, -w * sh;
    m_A.push_back(Anew);
    m_B.push_back(Bnew);
  }

  for(unsigned i = 0; i < window_.halfSize(); i++)
  {
    m_w[i] = wTable_(m_w2[i]);
  }

  // update future matrices
  for(unsigned i = window_.center(); i < window_.last(); i++)
  {
    const double w2 = m_w2[i];
    const double w = wTable_(w2);
    const double ch = chkTable_(w2);
    const double sh = shkTable_(w2);
    m_w[i] = w;
    m_A[i] << ch, sh / w, w * sh, ch;
    m_B[i] << 1 - ch, -w * sh;
  }

  unsigned lastIdx = window_.last();
  m_An[lastIdx] = m_A[lastIdx];
  m_Bn[lastIdx] = m_B[lastIdx];

  Vector2 Bsum(m_Bn[lastIdx] * m_p_ref(lastIdx));
  for(int i = static_cast<int>(lastIdx) - 1; i >= 0; i--)
  {
    unsigned idx = static_cast<unsigned>(i);
    m_An[idx].noalias() = m_An[idx + 1u] * m_A[idx];
    m_Bn[idx].noalias() = m_An[idx + 1u] * m_B[idx];
    Bsum.noalias() += m_Bn[idx] * m_p_ref(idx);
  }

  Vector2 Xg(m_p_ref(0), m_p_ref(lastIdx));
  Vector2 Vg;
  Vg << -(m_An[0](0, 0) * Xg(0) - Xg(1) + Bsum(0)) / m_An[0](0, 1),
      -m_An[0](1, 1) * (m_An[0](0, 0) * Xg(0) - Xg(1) + Bsum(0)) / m_An[0](0, 1) + m_An[0](1, 0) * Xg(0) + Bsum(1);

  m_X[0] << Xg(0), Vg(0); // set initial states
  for(unsigned i = 0; i < window_.last(); i++)
  {
    m_X[i + 1] = (m_A[i] * m_X[i] + m_B[i] * m_p_ref(i)).eval();
  }
}

void LinearTimeVariantInvertedPendulum::generate(Eigen::VectorXd & cog_pos,
                                                 Eigen::VectorXd & cog_vel,
                                                 Eigen::VectorXd & cog_acc,
                                                 Eigen::VectorXd & p_ref)
{
  // update future matrices
  for(unsigned i = 0; i < window_.size(); i++)
  {
    const double w2 = m_w2[i];
    const double w = wTable_(w2);
    const double ch = chkTable_(w2);
    const double sh = shkTable_(w2);
    m_w[i] = w;
    m_A[i] << ch, sh / w, w * sh, ch;
    m_B[i] << 1 - ch, -w * sh;
  }

  auto lastIdx = window_.last();
  m_An[lastIdx] = m_A[lastIdx];
  m_Bn[lastIdx] = m_B[lastIdx];

  Vector2 Bsum(m_Bn[lastIdx] * m_p_ref(lastIdx));
  for(int i = static_cast<int>(lastIdx) - 1; i >= 0; i--)
  {
    unsigned idx = static_cast<unsigned>(i);
    m_An[idx].noalias() = m_An[idx + 1u] * m_A[idx];
    m_Bn[idx].noalias() = m_An[idx + 1u] * m_B[idx];
    Bsum.noalias() += m_Bn[idx] * m_p_ref(idx);
  }

  Vector2 Xg(m_p_ref(0), m_p_ref(lastIdx));
  Vector2 Vg;
  Vg << -(m_An[0](0, 0) * Xg(0) - Xg(1) + Bsum(0)) / m_An[0](0, 1),
      -m_An[0](1, 1) * (m_An[0](0, 0) * Xg(0) - Xg(1) + Bsum(0)) / m_An[0](0, 1) + m_An[0](1, 0) * Xg(0) + Bsum(1);

  Vector2 X(Xg(0), Vg(0));
  for(unsigned i = 0; i < lastIdx; i++)
  {
    cog_pos(i) = X(0);
    cog_vel(i) = X(1);
    cog_acc(i) = m_w2[i] * (cog_pos(i) - m_p_ref(i));
    p_ref(i) = m_p_ref(i);

    X = (m_A[i] * X + m_B[i] * m_p_ref(i)).eval();
  }
  cog_pos(lastIdx) = X(0);
  cog_vel(lastIdx) = X(1);
  cog_acc(lastIdx) = m_w2[lastIdx] * (cog_pos(lastIdx) - m_p_ref(lastIdx));
  p_ref(lastIdx) = m_p_ref(lastIdx);
}

LinearTimeVariantInvertedPendulum::State LinearTimeVariantInvertedPendulum::getState(unsigned n_time) const
{
  const auto t = window_.center() + n_time;
  State s;
  s.p = m_p_ref(t);
  s.pdot = (s.p - m_p_ref[t - 1u]) / window_.dt();
  s.cog_pos = m_X[t](0);
  s.cog_vel = m_X[t](1);
  s.cog_acc = m_w2[t] * (s.cog_pos - s.p);
  return s;
}

void LinearTimeVariantInvertedPendulum::getState(unsigned n_time,
                                                 double & cog_pos,
                                                 double & cog_vel,
                                                 double & cog_acc,
                                                 double & p,
                                                 double & pdot)
{
  const auto t = window_.center() + n_time;
  p = m_p_ref(t);
  pdot = (p - m_p_ref[t - 1u]) / window_.dt();
  cog_pos = m_X[t](0);
  cog_vel = m_X[t](1);
  cog_acc = m_w2[t] * (cog_pos - p);
}

} // namespace linear_control_system

} // namespace mc_planning
