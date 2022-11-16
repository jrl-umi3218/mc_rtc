/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>

#include <string>

namespace mc_rbdyn
{

/*! \brief Stores mimic joint information */
struct MC_RBDYN_DLLAPI Mimic
{
public:
  /*! Name of the mimic joint */
  std::string name;
  /*! Which joint this joint mimics */
  std::string joint;
  /*! Mimic multiplier (usually -1/+1) */
  double multiplier;
  /*! Mimic offset */
  double offset;
};

inline bool operator==(const Mimic & lhs, const Mimic & rhs)
{
  return lhs.name == rhs.name && lhs.joint == rhs.joint && lhs.multiplier == rhs.multiplier && lhs.offset == rhs.offset;
}

} // namespace mc_rbdyn
