// g++ test_mat_mal.cpp -o test_mat -g -Wall -I.. -I../.. -I/usr/include/eigen3
// -I/home/morisawa/openrtp/include/OpenHRP-3.1 -lm -lhrpUtil-3.1

#include <mc_planning/LinearTimeInvariantInvertedPendulum.h>
#include <mc_rtc/constants.h>
#include <iostream>
// #include <Math/MathFunction.h>
// #include <Math/Physics.h>

namespace cst = mc_rtc::constants;

namespace mc_planning
{

LinearTimeVariantInvertedPendulum::LinearTimeVariantInvertedPendulum(void) {}

LinearTimeVariantInvertedPendulum::LinearTimeVariantInvertedPendulum(double dt, int n_preview, int weight_resolution)
{
  Initialize(dt, n_preview, weight_resolution);
}

void LinearTimeVariantInvertedPendulum::Initialize(double dt, int n_preview, int weight_resolution)
{
  m_dt = dt;
  m_n_current = n_preview;
  if(n_preview == 0)
  {
    m_n_current = lround(1.6 / m_dt);
  }
  m_n_preview2 = n_preview * 2 + 1;

  if(weight_resolution == 0)
  {
    weight_resolution = 20000;
  }

  m_wk.resize(weight_resolution);
  m_shk.resize(weight_resolution);
  m_chk.resize(weight_resolution);

  m_dh = (double)weight_resolution / 800.0;
  for(int i = 0; i < weight_resolution; i++)
  {
    double w2 = 0.001 + (double)i / m_dh;
    m_wk[i] = sqrt(w2);
    m_shk[i] = sinh(m_wk[i] * dt);
    m_chk[i] = cosh(m_wk[i] * dt);
  }

  m_A.resize(m_n_preview2, Matrix22::Identity());
  m_An.resize(m_n_preview2, Matrix22::Identity());
  m_B.resize(m_n_preview2, Vector2::Zero());
  m_Bn.resize(m_n_preview2, Vector2::Zero());

  m_X.resize(m_n_preview2, Vector2::Zero());
  m_p_ref.setZero(m_n_preview2);
  m_w2.setZero(m_n_preview2);
  m_w.setZero(m_n_preview2);
}

LinearTimeVariantInvertedPendulum::~LinearTimeVariantInvertedPendulum() {}

void LinearTimeVariantInvertedPendulum::init_m22(double waist_height)
{
  int h = lround(cst::GRAVITY * m_dh / waist_height);

  Matrix22 m;
  m << m_chk[h], m_shk[h] / m_wk[h], m_wk[h] * m_shk[h], m_chk[h];

  for(int i = 0; i < m_n_preview2; i++) m_A.push_front(m);
}

void LinearTimeVariantInvertedPendulum::init_v2(double waist_height)
{
  int h = lround(cst::GRAVITY * m_dh / waist_height);

  Vector2 v;
  v << 1 - m_chk[h], -m_wk[h] * m_shk[h];

  for(int i = 0; i < m_n_preview2; i++) m_B.push_front(v);
}

void LinearTimeVariantInvertedPendulum::initMatrices(double waist_height)
{
  init_m22(waist_height);
  init_v2(waist_height);
}

void LinearTimeVariantInvertedPendulum::update(void)
{
  // set newest matrices
  {
    int h = lround(m_w2[m_n_preview2 - 1] * m_dh);
    const double & w = m_wk[h];
    const double & ch = m_chk[h];
    const double & sh = m_shk[h];
    m_w[m_n_preview2 - 1] = w;
    Matrix22 Anew;
    Anew << ch, sh / w, w * sh, ch;
    Vector2 Bnew;
    Bnew << 1 - ch, -w * sh;
    m_A.push_back(Anew);
    m_B.push_back(Bnew);
  }

  for(int i = 0; i < m_n_current; i++)
  {
    int h = lround(m_w2[i] * m_dh);
    m_w[i] = m_wk[h];
  }

  // update future matrices
  for(int i = m_n_current; i < m_n_preview2 - 1; i++)
  {
    int h = lround(m_w2[i] * m_dh);
    const double & w = m_wk[h];
    const double & ch = m_chk[h];
    const double & sh = m_shk[h];
    m_w[i] = w;
    m_A[i] << ch, sh / w, w * sh, ch;
    m_B[i] << 1 - ch, -w * sh;
  }

  m_An[m_n_preview2 - 1] = m_A[m_n_preview2 - 1];
  m_Bn[m_n_preview2 - 1] = m_B[m_n_preview2 - 1];

  Vector2 Bsum(m_Bn[m_n_preview2 - 1] * m_p_ref(m_n_preview2 - 1));
  for(int i = m_n_preview2 - 2; i >= 0; i--)
  {
    m_An[i].noalias() = m_An[i + 1] * m_A[i];
    m_Bn[i].noalias() = m_An[i + 1] * m_B[i];
    Bsum.noalias() += m_Bn[i] * m_p_ref(i);
  }

  Vector2 Xg(m_p_ref(0), m_p_ref(m_n_preview2 - 1));
  Vector2 Vg;
  Vg << -(m_An[0](0, 0) * Xg(0) - Xg(1) + Bsum(0)) / m_An[0](0, 1),
      -m_An[0](1, 1) * (m_An[0](0, 0) * Xg(0) - Xg(1) + Bsum(0)) / m_An[0](0, 1) + m_An[0](1, 0) * Xg(0) + Bsum(1);

  m_X[0] << Xg(0), Vg(0); // set initial states
  for(int i = 0; i < m_n_preview2 - 1; i++) m_X[i + 1] = (m_A[i] * m_X[i] + m_B[i] * m_p_ref(i)).eval();
}

void LinearTimeVariantInvertedPendulum::generate(hrp::dvector & cog_pos,
                                                 hrp::dvector & cog_vel,
                                                 hrp::dvector & cog_acc,
                                                 hrp::dvector & p_ref)
{
  // update future matrices
  for(int i = 0; i < m_n_preview2; i++)
  {
    int h = lround(m_w2[i] * m_dh);
    const double & w = m_wk[h];
    const double & ch = m_chk[h];
    const double & sh = m_shk[h];
    m_w[i] = w;
    m_A[i] << ch, sh / w, w * sh, ch;
    m_B[i] << 1 - ch, -w * sh;
  }

  m_An[m_n_preview2 - 1] = m_A[m_n_preview2 - 1];
  m_Bn[m_n_preview2 - 1] = m_B[m_n_preview2 - 1];

  Vector2 Bsum(m_Bn[m_n_preview2 - 1] * m_p_ref(m_n_preview2 - 1));
  for(int i = m_n_preview2 - 2; i >= 0; i--)
  {
    m_An[i].noalias() = m_An[i + 1] * m_A[i];
    m_Bn[i].noalias() = m_An[i + 1] * m_B[i];
    Bsum.noalias() += m_Bn[i] * m_p_ref(i);
  }

  Vector2 Xg(m_p_ref(0), m_p_ref(m_n_preview2 - 1));
  Vector2 Vg;
  Vg << -(m_An[0](0, 0) * Xg(0) - Xg(1) + Bsum(0)) / m_An[0](0, 1),
      -m_An[0](1, 1) * (m_An[0](0, 0) * Xg(0) - Xg(1) + Bsum(0)) / m_An[0](0, 1) + m_An[0](1, 0) * Xg(0) + Bsum(1);

  Vector2 X(Xg(0), Vg(0));
  for(int i = 0; i < m_n_preview2 - 1; i++)
  {
    cog_pos(i) = X(0);
    cog_vel(i) = X(1);
    cog_acc(i) = m_w2[i] * (cog_pos(i) - m_p_ref(i));
    p_ref(i) = m_p_ref(i);

    X = (m_A[i] * X + m_B[i] * m_p_ref(i)).eval();
  }
  cog_pos(m_n_preview2 - 1) = X(0);
  cog_vel(m_n_preview2 - 1) = X(1);
  cog_acc(m_n_preview2 - 1) = m_w2[m_n_preview2 - 1] * (cog_pos(m_n_preview2 - 1) - m_p_ref(m_n_preview2 - 1));
  p_ref(m_n_preview2 - 1) = m_p_ref(m_n_preview2 - 1);
}

void LinearTimeVariantInvertedPendulum::getState(int n_time,
                                                 double & cog_pos,
                                                 double & cog_vel,
                                                 double & cog_acc,
                                                 double & p,
                                                 double & pdot)
{
  p = m_p_ref(m_n_current + n_time);
  pdot = (p - m_p_ref[m_n_current + n_time - 1]) / m_dt;
  cog_pos = m_X[m_n_current + n_time](0);
  cog_vel = m_X[m_n_current + n_time](1);
  cog_acc = m_w2[m_n_current + n_time] * (cog_pos - p);
}

void LinearTimeVariantInvertedPendulum::getState(int n_time,
                                                 double & cog_pos,
                                                 double & cog_vel,
                                                 double & cog_acc,
                                                 double & p)
{
  double pdot;
  getState(n_time, cog_pos, cog_vel, cog_acc, p, pdot);
}

void LinearTimeVariantInvertedPendulum::getState(int n_time, double & cog_pos, double & cog_vel, double & cog_acc)
{
  double p, pdot;
  getState(n_time, cog_pos, cog_vel, cog_acc, p, pdot);
}

void LinearTimeVariantInvertedPendulum::getState(int n_time, double & cog_pos, double & cog_vel)
{
  double cog_acc, p, pdot;
  getState(n_time, cog_pos, cog_vel, cog_acc, p, pdot);
}

void LinearTimeVariantInvertedPendulum::getState(int n_time, double & cog_pos)
{
  double cog_vel, cog_acc, p, pdot;
  getState(n_time, cog_pos, cog_vel, cog_acc, p, pdot);
}

} // namespace mc_planning
