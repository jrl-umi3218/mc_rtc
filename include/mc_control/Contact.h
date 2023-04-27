/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>

#include <mc_rbdyn/Contact.h>

#include <mc_rtc/Configuration.h>

namespace mc_control
{

struct MCController;

/** \class Contact
 *
 * A lightweight variant of mc_rbdyn::Contact meant to simplify contact manipulations
 *
 * If \ref r1 or \ref r2 is std::nullopt then the controller will use the main robot instead
 *
 */
struct MC_CONTROL_DLLAPI Contact
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Contact(const std::optional<std::string> & r1,
          const std::optional<std::string> & r2,
          const std::string & r1Surface,
          const std::string & r2Surface,
          double friction = mc_rbdyn::Contact::defaultFriction,
          const Eigen::Vector6d & dof = Eigen::Vector6d::Ones())
  : r1(r1), r2(r2), r1Surface(r1Surface), r2Surface(r2Surface), friction(friction), dof(dof)
  {
  }

  std::optional<std::string> r1;
  std::optional<std::string> r2;
  std::string r1Surface;
  std::string r2Surface;
  mutable double friction;
  mutable Eigen::Vector6d dof;

  bool operator==(const Contact & rhs) const
  {
    return (r1 == rhs.r1 && r2 == rhs.r2 && r1Surface == rhs.r1Surface && r2Surface == rhs.r2Surface)
           || (r1 == rhs.r2 && r2 == rhs.r1 && r1Surface == rhs.r2Surface && r2Surface == rhs.r1Surface);
  }

  bool operator!=(const Contact & rhs) const { return !(*this == rhs); }

  /** Default constructor, invalid contact */
  Contact() = default;

  /** Conversion from mc_rbdyn::Contact */
  static Contact from_mc_rbdyn(const MCController &, const mc_rbdyn::Contact &);
};

using ContactSet =
    std::unordered_set<Contact, std::hash<Contact>, std::equal_to<Contact>, Eigen::aligned_allocator<Contact>>;

} // namespace mc_control

namespace std
{

template<>
struct hash<mc_control::Contact>
{
  std::size_t operator()(const mc_control::Contact & c) const noexcept
  {
    // Same as boost::hash_combine
    auto hash_combine = [](const std::string & robot, const std::string & surface)
    {
      auto h = std::hash<std::string>{}(robot);
      h ^= std::hash<std::string>{}(surface) + 0x9e3779b9 + (h << 6) + (h >> 2);
      return h;
    };
    return hash_combine(c.r1.value_or(""), c.r1Surface) ^ hash_combine(c.r2.value_or(""), c.r2Surface);
  }
};

} // namespace std

namespace mc_rtc
{

template<>
struct MC_CONTROL_DLLAPI ConfigurationLoader<mc_control::Contact>
{
  static mc_control::Contact load(const mc_rtc::Configuration & config);
};

} // namespace mc_rtc
