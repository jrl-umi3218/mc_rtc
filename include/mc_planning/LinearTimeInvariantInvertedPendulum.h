/*
 * Copyright (c) 2017,
 * @author Mitsuharu Morisawa
 *
 * AIST
 *
 * All rights reserved.
 *
 * This program is made available under the terms of the Eclipse Public License
 * v1.0 which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 */
#pragma once

#include <boost/circular_buffer.hpp>
#include <Eigen/Core>
#include <vector>

// XXX to get rid of
namespace hrp
{
typedef Eigen::Vector2d Vector2;
typedef Eigen::Vector3d Vector3;
typedef Eigen::Matrix3d Matrix33;
typedef Eigen::MatrixXd dmatrix;
typedef Eigen::VectorXd dvector;
typedef Eigen::VectorXi ivector;
typedef Eigen::Matrix<double, 6, 1> dvector6;
typedef Eigen::Quaternion<double> dquaternion;
} // namespace hrp

namespace mc_planning
{

class LinearTimeVariantInvertedPendulum
{
protected:
  using Matrix22 = Eigen::Matrix<double, 2, 2>;
  using Vector2 = Eigen::Matrix<double, 2, 1>;

  double m_dt;
  double m_dh;
  int m_n_current;
  int m_n_preview2;

  std::vector<double> m_wk;
  std::vector<double> m_shk;
  std::vector<double> m_chk;

  boost::circular_buffer<Matrix22, Eigen::aligned_allocator<Matrix22>> m_A;
  boost::circular_buffer<Vector2, Eigen::aligned_allocator<Vector2>> m_B;
  std::vector<Matrix22, Eigen::aligned_allocator<Matrix22>> m_An;
  std::vector<Vector2, Eigen::aligned_allocator<Vector2>> m_Bn;

  std::vector<Vector2, Eigen::aligned_allocator<Vector2>> m_X;
  hrp::dvector m_p_ref;
  hrp::dvector m_w2;
  hrp::dvector m_w;

private:
  inline void init_m22(double waist_height);
  inline void init_v2(double waist_height);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  hrp::dvector & p_ref(void)
  {
    return m_p_ref;
  }
  double & p_ref(int i)
  {
    return m_p_ref[i];
  }
  hrp::dvector & w2(void)
  {
    return m_w2;
  }
  double & w2(int i)
  {
    return m_w2[i];
  }
  hrp::dvector & w(void)
  {
    return m_w;
  }
  double & w(int i)
  {
    return m_w[i];
  }

  LinearTimeVariantInvertedPendulum();
  LinearTimeVariantInvertedPendulum(double dt, int n_preview = 0, int weight_resolution = 20000);

  virtual ~LinearTimeVariantInvertedPendulum();

  void Initialize(double dt, int n_preview = 0, int weight_resolution = 20000);

  void initMatrices(double waist_height);

  void setOmega(int n_start, int n_end);

  void update(void);

  /*! @brief to get generated CoG/ZMP states
   * @param[in]  n_time is the time in range of[-n_preview:n_preview], current time is 0
   * @param[out]  cog_pos is position of COG
   * @param[out]  cog_vel is velocity of COG
   * @param[out]  cog_acc is acceleration of COG
   * @param[in]  p is position of ZMP
   * @param[in]  pdot is velocity of ZMP
   */
  void getState(int n_time, double & cog_pos, double & cog_vel, double & cog_acc, double & p, double & pdot);
  void getState(int n_time, double & cog_pos, double & cog_vel, double & cog_acc, double & p);
  void getState(int n_time, double & cog_pos, double & cog_vel, double & cog_acc);
  void getState(int n_time, double & cog_pos, double & cog_vel);
  void getState(int n_time, double & cog_pos);

  void generate(hrp::dvector & cog_pos, hrp::dvector & cog_vel, hrp::dvector & cog_acc, hrp::dvector & p_ref);
};

} // namespace mc_planning
