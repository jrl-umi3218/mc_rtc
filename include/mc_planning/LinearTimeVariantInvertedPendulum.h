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

#include <mc_planning/LookupTable.h>
#include <mc_planning/PreviewWindow.h>
#include <mc_planning/api.h>
#include <mc_rtc/logging.h>
#include <boost/circular_buffer.hpp>
#include <Eigen/Core>
#include <vector>

namespace mc_planning
{

namespace linear_control_system
{

/**
 * @brief Reference CoM position and velocity trajectory generation.
 *
 * Implementation of the Linear Time-Varying System (LTVS) introduced in Sec III.A of:
 *
 * **Online 3D CoM Trajectory Generation for Multi-Contact Locomotion Synchronizing Contact**, *Mitsuharu Morisawa et
 * al., IROS 2018*
 *
 * Computes the horizontal long-term trajectory of the CoM along either sagittal or frontal plane.
 *
 * The discretized system of the centroidal dynamics in the sagital plane is obtained as
 *
 * \f[
 *   x_{k+1} = A_k x_k + B_k u_k
 * \f]
 *
 * where
 *
 * - \f[A_k = \begin{bmatrix}
 *              cosh(\omega_k\delta T) & \frac{sinh(\omega_k\delta T)}{\omega_k} \\
 *              \omega_k sinh(\omega_k\delta T) & cosh(\omega_k \delta T)
 *            \end{bmatrix}
 *   \f]
 * - CoM state position/velocity along the sagital plane
 *   \f[ x_k = [x_{G,k}, \dot{x}_{G,k}] \f]
 * - \f[ B_k = \begin{bmatrix}
 *              1- cosh(\omega_k\delta T) \\
 *              - \omega_k sinh(\omega_k\delta T)
 *            \end{bmatrix}
 *   \f]
 * - Desired Centroidal Momentum Pivot (CMP) [equivalent to the ZMP]. \f$ L \f$ is the number of contact links.
 *   \f[ u_k = \sum^L_{i=1} \alpha_{z_i,k}p_{x_i,k} - \frac{\sigma_{y,k}}{m(g+\ddot{z}_{G,k})} \f]
 * - \f[
 *   \omega_k =
 *   \sqrt{\frac{g+\ddot{z}_{G,k}}{z_{G,k}-\sum_{i=1}^{L}\alpha_{x-i,k}p_{z_i,l}}}
 *   \f]
 *
 * This system is used to compute the reference position and velocity of the CoM
 * for a preview window:
 * - of size \f$ N_p = 2*N+1 \f$ centered around the current time \f$ [-N,\mbox{current},N] \f$
 * - this is represented internally by arrays indexed from \f$ [0...M] \f$ with \f$ M = 2*N \f$ with the current time
 * being at index \f$ N \f$
 *
 * **Expects the following to be defined externally**:
 * - \f$ [\omega^2_0...\omega_M^2] \f$ defined by w2()
 * - Desired CMP trajectory: \f$ [u_0...u_M] \f$ defined by p_ref()
 *
 * The CoM state \f$ x_F \f$ after the F-th future step is
 *
 * \f[
 * x_F = \Phi(F,0)x_0 + \sum_{i=0}^{F-1}\Phi(F,i+1)B_i u_i
 * \f]
 *
 * where
 *
 * \f[
 * \Phi(k,j) =
 *   \left\{
 *    \begin{array}{ll}
 *        A_{k-1}A_{k-2}...A_j &\mbox{if } k>j \\
 *        I_2 & \mbox{otherwise }(k=j=F)
 *      \end{array}
 *    \right.
 *  \f]
 *
 * Those are computed by update() as m_An, m_Bn, and the corresponding trajectory as m_X
 */
struct MC_PLANNING_DLLAPI LinearTimeVariantInvertedPendulum
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Initialization
   *
   * @param dt Timestep
   * @param n_preview Number of future preview elements. The full size of the
   * preview windows from past to future will be (2*n_preview+1)
   * @param minHeight for the pendulum (used for LookupTable optimizations)
   * @param maxHeight for the pendulum
   * @param weight_resolution Number of data points to pre-compute for
   * \f$ \omega, cosh(\omega), sinh(\omega) \f$ for height in a given range
   * [minHeight, maxHeight]. This is used to optimize the otherwise costly computation of these
   * functions.
   */
  LinearTimeVariantInvertedPendulum(const CenteredPreviewWindow & window,
                                    unsigned weight_resolution = 20000,
                                    double minHeight = 0.01,
                                    double maxHeight = 2.5);

  /**
   * @brief Initialize the discretized system matrices with constant pendulum height
   *
   * Initalizes \f$ A_k \f$, \f$ B_k \f$
   *
   * @param waist_height Pendulum height (constant over the preview window)
   */
  void initMatrices(double waist_height);

  /**
   * @brief Computes the reference CoM state online (position and velocity)
   *
   * Fills m_A, m_B, m_An, m_Bn, m_X
   * Requires p_ref() and w2() to be filled first.
   */
  void update();

  /**
   * @brief Generated CoM/ZMP state
   */
  struct State
  {
    double cog_pos; ///< CoM position
    double cog_vel; ///< CoM velocity
    double cog_acc; ///< CoM acceleration
    double p; ///< ZMP position
    double pdot; ///< ZMP velocity
  };

  /*! @brief Get generated CoG/ZMP states
   * @param n_time is the time in range of[-n_preview:n_preview], current time is 0
   */
  State getState(unsigned n_time) const;

  /*! @brief to get generated CoG/ZMP states
   * @param[in]  n_time is the time in range of[-n_preview:n_preview], current time is 0
   * @param[out]  cog_pos is position of COG
   * @param[out]  cog_vel is velocity of COG
   * @param[out]  cog_acc is acceleration of COG
   * @param[in]  p is position of ZMP
   * @param[in]  pdot is velocity of ZMP
   */
  void getState(unsigned n_time, double & cog_pos, double & cog_vel, double & cog_acc, double & p, double & pdot);

  /**
   * Generate full trajectory (only used for tests)
   */
  void generate(Eigen::VectorXd & cog_pos,
                Eigen::VectorXd & cog_vel,
                Eigen::VectorXd & cog_acc,
                Eigen::VectorXd & p_ref);

  /** Reference CMP along the trajectory \f$ [u_0..u_M] \f$
   *
   * @return Reference to the CMP array
   */
  Eigen::VectorXd & p_ref()
  {
    return m_p_ref;
  }

  /**
   * @brief Value of \f$ u_k \f$
   *
   * @param k Index of the value in range \f$ [0, M] \f$
   */
  double p_ref(unsigned k)
  {
    return m_p_ref[k];
  }

  /** Squared values of \f$ \omega_k \f$ along the preview time: \f$ [\omega_0^2 ... \omega_M^2] \f$
   */
  Eigen::VectorXd & w2()
  {
    return m_w2;
  }

  /**
   * @brief Value of \f$ \omega_k^2 \f$
   *
   * @param k Index of the value in range \f$ [0, M] \f$
   */
  double & w2(unsigned k)
  {
    return m_w2[k];
  }
  const Eigen::VectorXd & w() const
  {
    return m_w;
  }
  /**
   * @brief Value of \f$ \omega_k \f$
   *
   * @param k Index of the value in range \f$ [0, M] \f$
   */
  double w(unsigned k) const
  {
    return m_w[k];
  }

protected:
  const CenteredPreviewWindow window_;
  using Matrix22 = Eigen::Matrix<double, 2, 2>;
  using Vector2 = Eigen::Matrix<double, 2, 1>;

  boost::circular_buffer<Matrix22, Eigen::aligned_allocator<Matrix22>> m_A; ///< \f$ A_k \f$
  boost::circular_buffer<Vector2, Eigen::aligned_allocator<Vector2>> m_B; ///< \f$ B_k \f$
  std::vector<Matrix22, Eigen::aligned_allocator<Matrix22>> m_An; ///< \f$ \Phi(k, j) \f$
  std::vector<Vector2, Eigen::aligned_allocator<Vector2>> m_Bn; ///< \f$ \sum_{i=0}^{F-1}\Phi(F,i+1)B_i \f$
  std::vector<Vector2, Eigen::aligned_allocator<Vector2>>
      m_X; ///< Generated trajectory for the CoM (position, velocity)

  Eigen::VectorXd m_p_ref; ///< \f$ [u_0,...,u_M] \f$

  Eigen::VectorXd m_w2; ///< \f$ \omega_k^2 \f$
  Eigen::VectorXd m_w; ///< \f$ \omega \f$

  /**
   * @name Lookup tables for optimized computation of cosh, sinh and sqrt
   **/
  ///@{
  LookupTable<double> wTable_; ///< Pre-computed table from \f$ \omega^2 \rightarrow sqrt(\omega^2) \f$
  LookupTable<double> chkTable_; ///< Pre-computed table from \f$ \omega^2 \rightarrow cosh(sqrt(\omega^2) * dt) \f$
  LookupTable<double> shkTable_; ///< Pre-computed table from \f$ \omega^2 \rightarrow sinh(sqrt(\omega^2) * dt) \f$
  ///@}
};

} // namespace linear_control_system

} // namespace mc_planning
