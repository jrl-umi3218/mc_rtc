/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/api.h>

#include <mc_rtc/Configuration.h>

namespace mc_tasks
{

namespace force
{

namespace details
{

/** A wrapper around an sva::ImpedanceVecd that imposes (strictly) positive values */
template<bool StrictlyPositive>
struct ImpedanceVecd
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImpedanceVecd() : ImpedanceVecd(limit, limit) {}

  ImpedanceVecd(const sva::ImpedanceVecd & v)
  {
    vec(v);
  }

  ImpedanceVecd(double angular_, double linear_)
  {
    angular(angular_);
    linear(linear_);
  }

  ImpedanceVecd & operator=(const sva::ImpedanceVecd & v)
  {
    vec(v);
    return *this;
  }

  inline Eigen::Vector6d vector() const noexcept
  {
    return vec_.vector();
  }

  inline void vec(const sva::ImpedanceVecd & v) noexcept
  {
    angular(v.angular());
    linear(v.linear());
  }

  inline void vec(const Eigen::Vector6d & v) noexcept
  {
    angular(v.head<3>());
    linear(v.tail<3>());
  }

  inline void vec(const Eigen::Vector3d & a, const Eigen::Vector3d & l) noexcept
  {
    angular(a);
    linear(l);
  }

  inline void vec(double a, double l) noexcept
  {
    angular(a);
    linear(l);
  }

  inline const sva::ImpedanceVecd & vec() const noexcept
  {
    return vec_;
  }

  inline void angular(const Eigen::Vector3d & v) noexcept
  {
    vec_.angular() = v.cwiseMax(limit);
  }

  inline void angular(double angular) noexcept
  {
    vec_.angular().setConstant(std::max(angular, limit));
  }

  inline const Eigen::Vector3d & angular() const noexcept
  {
    return vec_.angular();
  }

  inline void linear(const Eigen::Vector3d & v) noexcept
  {
    vec_.linear() = v.cwiseMax(limit);
  }

  inline void linear(double linear) noexcept
  {
    vec_.linear().setConstant(std::max(linear, limit));
  }

  inline const Eigen::Vector3d & linear() const noexcept
  {
    return vec_.linear();
  }

private:
  static constexpr double limit = StrictlyPositive ? 1e-6 : 0.0;
  sva::ImpedanceVecd vec_;
};

// Repeat static constexpr declarations
// See also https://stackoverflow.com/q/8016780
template<bool b>
constexpr double ImpedanceVecd<b>::limit;

} // namespace details

/*! \brief Represent impedance gains for an \ref ImpedanceTask */
struct MC_TASKS_DLLAPI ImpedanceGains
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Impedance mass parameter */
  inline const details::ImpedanceVecd<true> & M() const noexcept
  {
    return M_;
  }
  /** Impedance mass parameter */
  inline details::ImpedanceVecd<true> & M() noexcept
  {
    return M_;
  }
  /** Impedance mass parameter */
  inline const details::ImpedanceVecd<true> & mass() const noexcept
  {
    return M_;
  }
  /** Impedance mass parameter */
  inline details::ImpedanceVecd<true> & mass() noexcept
  {
    return M_;
  }

  /** Impedance damper parameter */
  inline const details::ImpedanceVecd<false> & D() const noexcept
  {
    return D_;
  }
  /** Impedance damper parameter */
  inline details::ImpedanceVecd<false> & D() noexcept
  {
    return D_;
  }
  /** Impedance damper parameter */
  inline const details::ImpedanceVecd<false> & damper() const noexcept
  {
    return D_;
  }
  /** Impedance damper parameter */
  inline details::ImpedanceVecd<false> & damper() noexcept
  {
    return D_;
  }

  /** Impedance spring parameter */
  inline const details::ImpedanceVecd<false> & K() const noexcept
  {
    return K_;
  }
  /** Impedance spring parameter */
  inline details::ImpedanceVecd<false> & K() noexcept
  {
    return K_;
  }
  /** Impedance spring parameter */
  inline const details::ImpedanceVecd<false> & spring() const noexcept
  {
    return K_;
  }
  /** Impedance spring parameter */
  inline details::ImpedanceVecd<false> & spring() noexcept
  {
    return K_;
  }

  /** Impedance wrench gain */
  inline const details::ImpedanceVecd<false> & wrench() const noexcept
  {
    return wrench_;
  }
  /** Impedance wrench gain */
  inline details::ImpedanceVecd<false> & wrench() noexcept
  {
    return wrench_;
  }

  /** Returns default gains for the ImpedanceTask */
  inline static ImpedanceGains Default()
  {
    ImpedanceGains out;
    out.mass().vec(10.0, 10.0);
    out.damper().vec(1000.0, 1000.0);
    out.spring().vec(1000.0, 1000.0);
    return out;
  }

protected:
  /** Impedance mass parameter */
  details::ImpedanceVecd<true> M_;
  /** Impedance damper parameter */
  details::ImpedanceVecd<false> D_;
  /** Impedance spring parameter */
  details::ImpedanceVecd<false> K_;
  /** Impedance wrench gain */
  details::ImpedanceVecd<false> wrench_;
};

} // namespace force

} // namespace mc_tasks

namespace mc_rtc
{

template<>
struct MC_TASKS_DLLAPI ConfigurationLoader<mc_tasks::force::ImpedanceGains>
{
  static mc_tasks::force::ImpedanceGains load(const mc_rtc::Configuration & config)
  {
    mc_tasks::force::ImpedanceGains out;
    if(config.has("mass"))
    {
      sva::ImpedanceVecd M = config("mass");
      out.M().vec(M);
    }
    if(config.has("damper"))
    {
      sva::ImpedanceVecd D = config("damper");
      out.D().vec(D);
    }
    if(config.has("spring"))
    {
      sva::ImpedanceVecd K = config("spring");
      out.K().vec(K);
    }
    if(config.has("wrench"))
    {
      sva::ImpedanceVecd wrench = config("wrench");
      out.wrench().vec(wrench);
    }
    return out;
  }
  static mc_rtc::Configuration save(const mc_tasks::force::ImpedanceGains & ig)
  {
    mc_rtc::Configuration out;
    out.add("mass", ig.M().vec());
    out.add("damper", ig.D().vec());
    out.add("spring", ig.K().vec());
    out.add("wrench", ig.wrench().vec());
    return out;
  }
};

} // namespace mc_rtc
