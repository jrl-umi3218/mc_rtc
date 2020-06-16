/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <Eigen/Core>
#include <type_traits>

namespace mc_rtc
{
namespace internal
{
namespace is_eigen_matrix_detail
{
// These functions are never defined.
template<typename T>
std::true_type test(const Eigen::MatrixBase<T> *);
std::false_type test(...);
} // namespace is_eigen_matrix_detail
template<typename T>
struct is_eigen_matrix : public decltype(is_eigen_matrix_detail::test(std::declval<T *>()))
{
};
} // namespace internal
} // namespace mc_rtc
