/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_solver/QPSolver.h>
#include <mc_solver/api.h>

namespace mc_solver
{

namespace utils
{

/** Transform contact wrench matrix to lambda matrix
 *
 * This transforms a given matrix A that applies to a wrench (e.g. such as in
 * \f$A F <= F_{max}\f$) into a matrix that applies to lambdas (i.e. such \f$A'
 * \boldsymbol{\lambda} <= F_{max}\f$)
 *
 * For a given force \f$f_p\f$ at point \f$p\f$, the linearized friction cone
 * provides:
 *
 * \f[f_p = \sum^{N^{Gen}_{p}}_{i=1} G^{p}_{i} \lambda^{p}_{i}\f]
 *
 * Where \f$G^{p}_{i} \in \mathbb{R}^{3,1}\f$ are the generatrix of the friction cone. In the remainder of these
 * equations we write \f$N = N^{Gen}_{p}\f$ for sparcity
 *
 * In matrix form:
 *
 * \f[
 *  f_p = \begin{bmatrix}
 *          G^p_{1x} & \cdots & G^p_{Nx} \\
 *          G^p_{1y} & \cdots & G^p_{Ny} \\
 *          G^p_{1z} & \cdots & G^p_{Nz}
 *        \end{bmatrix} \begin{bmatrix}
 *          \lambda^{p}_{i} \\
 *          \vdots \\
 *          \lambda^{p}_{N}
 *        \end{bmatrix}
 * \f]
 *
 * For a given wrench \f$F\f$ at frame b, resulting from \f$P\f$ contacts
 *
 * \f{align}
 *  F & = \sum^{P}_{i=1} {}^{p_{i}}X_{b}^{T} f_{p_{i}} \\
 *    & = \begin{bmatrix}
 *            \begin{bmatrix}
 *              r^{\times}_{1}E^{T}_{1} \\
 *              E^{T}_{1}
 *            \end{bmatrix} \begin{bmatrix}
 *                            G^{1}_{1} & \cdots & G^{1}_{N}
 *                          \end{bmatrix}
 *            & \cdots &
 *            \begin{bmatrix}
 *              r^{\times}_{P}E^{T}_{P} \\
 *              E^{T}_{P}
 *            \end{bmatrix} \begin{bmatrix}
 *                            G^{P}_{1} & \cdots & G^{P}_{N}
 *                          \end{bmatrix}
 *          \end{bmatrix}
 *          \begin{bmatrix}
 *            \lambda^{1}_{1} \\
 *            \vdots \\
 *            \lambda^{P}_{N}
 *          \end{bmatrix}
 * \f}
 *
 * Where \f${}^{p_{i}}X_{b}\f$ is the Plücker transform from the point \f$p\f$ to the wrench frame \f$b\f$:
 *
 * \f{align*}
 *  {}^{p_{i}}X_{b} = \begin{bmatrix}
 *                      E_{i}       & 0 \\
 *                      -E_{i}r_{i}^{\times} & E_{i}
 *                    \end{bmatrix}
 *  &
 *
 *  &
 *  {}^{p_{i}}X_{b}^{T} = \begin{bmatrix}
 *                      E^{T}_{i}       & r^{\times}_{i}E^{T}_{i} \\
 *                      0 & E^{T}_{i}
 *                    \end{bmatrix}
 * \f}
 *
 * Note: we only take the last three columns on \f${}^{p_{i}}X_{b}^{T}\f$ in Equation (2)
 *
 * Hence:
 *
 * \f{align*}
 *  A' = A \begin{bmatrix}
 *           \begin{bmatrix}
 *             r^{\times}_{1}E^{T}_{1} \\
 *             E^{T}_{1}
 *           \end{bmatrix} \begin{bmatrix}
 *                           G^{1}_{1} & \cdots & G^{1}_{N}
 *                         \end{bmatrix}
 *           & \cdots &
 *           \begin{bmatrix}
 *             r^{\times}_{P}E^{T}_{P} \\
 *             E^{T}_{P}
 *           \end{bmatrix} \begin{bmatrix}
 *                           G^{P}_{1} & \cdots & G^{P}_{N}
 *                         \end{bmatrix}
 *         \end{bmatrix}
 * \f}
 */
struct MC_SOLVER_DLLAPI ContactWrenchMatrixToLambdaMatrix
{
  /** Creates the transformation matrix for a given contact
   *
   * \throws If the contact has not been added to the solver
   */
  ContactWrenchMatrixToLambdaMatrix(const mc_solver::QPSolver & solver, const tasks::qp::ContactId & cid);

  /** Compute the \f$A'\f$ matrix given the \f$A\f$ matrix */
  inline Eigen::MatrixXd transform(const Eigen::MatrixXd & A) const
  {
    return A * transform_;
  }

  const Eigen::MatrixXd & transform() const
  {
    return transform_;
  }

private:
  Eigen::MatrixXd transform_;
};

} // namespace utils

} // namespace mc_solver
