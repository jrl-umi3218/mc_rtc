/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_planning/RangeElementIterator.h>
#include <mc_planning/api.h>
#include <cmath>

namespace mc_planning
{

struct CenteredPreviewWindow;

/**
 * @brief Element returned by CenteredPreviewWindow::iterator providing the current
 * index and allowing to conveniently get the corresponding time
 */
struct MC_PLANNING_DLLAPI PreviewElement
{
  explicit PreviewElement(const CenteredPreviewWindow & window, unsigned index) noexcept
  : window_(window), index_(index)
  {
  }

  unsigned index() const noexcept
  {
    return index_;
  }

  double time() const noexcept;

protected:
  const CenteredPreviewWindow & window_;
  unsigned index_;
};

/**
 * @brief Iterable range over a preview window with convenience time-management.
 *
 * The preview window is centered around the current element \f$ [-N, current, N] \f$ where `N` is the number of
 * elements before and after the current value. The full preview window is of size `2*N+1`. The timestep is used to
 * compute correspondances between discrete index of values stored in an (external) container and time.
 *
 * \code{.cpp}
 *   // Create an iterable preview window range with a window of time `2*1.6`,
 *   // timestep of 0.005 and starting at time `t=0.5`
 *   CenteredPreviewWindow window(1.6, 0.005, 0.5);
 *   mc_rtc::log::info("Start time:   {:.3f}, index: {}\n"
 *                     "End time:     {:.3f}, index: {}\n"
 *                     "Current time: {:.3f}, index: {}",
 *                     window.startTime(),    window.startIndex(),
 *                     window.endTime(),      window.endIndex(),
 *                     window.currentTime(),  window.currentIndex());
 *
 *   // Iterate over the preview window using a RangeElementIterator
 *   for(const auto & w : window)
 *   {
 *     mc_rtc::log::info("Preview element with index: {}, time: {:.3f}", w.index(), w.time());
 *     // Example: Access some external container using the index
 *     // auto value = container[w.index()];
 *   }
 * \endcode
 *
 * \see For practical use, see:
 * \see mc_planning::generator
 * \see mc_planning::linear_control_system::LinearTimeVariantInvertedPendulum
 */
struct MC_PLANNING_DLLAPI CenteredPreviewWindow
{
  using iterator = RangeElementIterator<PreviewElement, CenteredPreviewWindow, false>;
  using const_iterator = RangeElementIterator<PreviewElement, CenteredPreviewWindow, true>;

  /**
   * @brief Constructs a window of duration `2*halfTime` with the current
   * element at the center.
   *
   * @param halfTime Half-duration of the window
   * @param dt Timesep
   * @param startTime Time at which the window starts
   */
  explicit CenteredPreviewWindow(double halfTime, double dt, const double startTime = 0.) noexcept
  : halfSize_(static_cast<unsigned>(std::lround(halfTime / dt))), fullSize_(2 * halfSize_ + 1), dt_(dt)
  {
    startAt(startTime);
  }

  /**
   * @name Range iterators for computing index/time along the preview window
   * @{
   */
  iterator begin() noexcept
  {
    return iterator{*this, startIndex()};
  }
  iterator end() noexcept
  {
    return iterator{*this, endIndex()};
  }

  const_iterator rbegin() noexcept
  {
    return const_iterator{*this, startIndex()};
  }
  const_iterator rend() noexcept
  {
    return const_iterator{*this, endIndex()};
  }
  // @}

  /**
   * @brief Start the preview window at the provided time
   *
   * @param t Time at which the window starts
   */
  void startAt(double t) noexcept
  {
    start_ = indexFromTime(t);
  }

  /**
   * @brief Time at the start of the preview window
   */
  inline double startTime() const noexcept
  {
    return start_ * dt_;
  }

  /**
   * @brief Index of the start of the preview window
   */
  unsigned startIndex() const noexcept
  {
    return start_;
  }

  /**
   * @brief Time at the center of the preview window
   */
  double currentTime() const noexcept
  {
    return (start_ + halfSize_) * dt_;
  }

  /**
   * @brief Index of the center of the preview window
   */
  unsigned currentIndex() const noexcept
  {
    return start_ + halfSize_;
  }

  /**
   * @brief Index of the end of the preview window
   */
  double endTime() const noexcept
  {
    return (start_ + fullSize_ - 1) * dt_;
  }

  /**
   * @brief Index of the end of the preview window
   */
  unsigned endIndex() const noexcept
  {
    return start_ + fullSize_ - 1;
  }

  /**
   * @brief Converts between time and index
   *
   * @param t Time
   *
   * @return Index corresponding to the provided time
   */
  unsigned indexFromTime(double t) const noexcept
  {
    return static_cast<unsigned>(std::lround(t / dt_));
  }

  /**
   * @brief Converts between index and time
   *
   * @param index Index
   *
   * @return Time corresponding to the provided index
   */
  double timeFromIndex(unsigned index) const noexcept
  {
    return index * dt_;
  }

protected:
  unsigned halfSize_ = 0; ///< Number of future or past element
  unsigned fullSize_ = 0; ///< 2*halfSize_+1
  unsigned start_ = 0; ///< Index at which the window starts
  double dt_ = 0.005; ///< Timestep
};

} // namespace mc_planning
