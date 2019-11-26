/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/gui/types.h>

namespace mc_rtc
{

namespace gui
{

/** Trajectory can represent a pre-defined trajectory or a real-time trajectory
 * depending on type of data returned by the callback
 *
 * Styling information is provided to inform the client about how to display the trajectory
 *
 * \tparam GetT Should return either an sva::PTransformd or an Eigen::Vector3d
 * (real-time trajectory) or a vector of these types (pre-defined trajectory)
 *
 */
template<typename GetT>
struct TrajectoryImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Trajectory;

  TrajectoryImpl(const std::string & name, const LineConfig & config, GetT get_fn)
  : DataElement<GetT>(name, get_fn), config_(config)
  {
    static_assert(details::CheckReturnType<GetT, Eigen::Vector3d, sva::PTransformd, std::vector<Eigen::Vector3d>,
                                           std::vector<sva::PTransformd>>::value,
                  "Trajectory element data callback must return either an Eigen::Vector3d, an sva::PTransformd or an "
                  "std::vector of either types");
  }

  /** Invalid element */
  TrajectoryImpl() {}

  constexpr static size_t write_size()
  {
    return DataElement<GetT>::write_size() + LineConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    DataElement<GetT>::write(builder);
    config_.write(builder);
  }

private:
  LineConfig config_;
};

/** Function helper to get a TrajectoryImpl */
template<typename GetT>
TrajectoryImpl<GetT> Trajectory(const std::string & name, GetT get_fn)
{
  return TrajectoryImpl<GetT>(name, {}, get_fn);
}

/** Function helper to get a TrajectoryImpl */
template<typename GetT>
TrajectoryImpl<GetT> Trajectory(const std::string & name, const LineConfig & config, GetT get_fn)
{
  return TrajectoryImpl<GetT>(name, config, get_fn);
}

} // namespace gui

} // namespace mc_rtc
