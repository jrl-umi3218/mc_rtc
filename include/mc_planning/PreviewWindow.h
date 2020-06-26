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
  /**
   * @brief Create an element (meant to be used by iterators)
   *
   * @param window Preview window to which this element refers to
   * @param index Global index of the element
   */
  explicit PreviewElement(const CenteredPreviewWindow & window, unsigned index) noexcept
  : window_(window), index_(index)
  {
  }

  /**
   * @brief Returns the global index starting at `window.startIndex()`
   */
  unsigned index() const noexcept;
  /**
   * @brief Returns the time corresponding to index()
   */
  double time() const noexcept;

  /**
   * @brief Returns the index local to the preview window
   *
   * @return Index between \f$ [0, 2*N] \f$ where `N` is the preview size
   */
  unsigned localIndex() const noexcept;

  /**
   * @brief Returns the time corresponding to localIndex()
   */
  double localTime() const noexcept;

  /**
   * @brief Reference to the preview window from which this element was
   * generated
   */
  const CenteredPreviewWindow & window() const noexcept
  {
    return window_;
  }

protected:
  const CenteredPreviewWindow & window_; ///< Window being iterated over
  unsigned index_; ///< Global index
};

/**
 * @brief Iterable range over a preview window with convenience time-management.
 *
 * The preview window is centered around the current element \f$ [-N, current, N] \f$ where `N` is the number of
 * elements before and after the current value. The full preview window is of size `2*N+1`. The timestep is used to
 * compute correspondances between discrete index of values stored in an (external) container and time.
 *
 * Additionally, this class provides iterable ranges over the preview window.
 * For example:
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
 *
 *   // Iterate over a copy of the preview window (cheap copy) starting at a specific time using a RangeElementIterator
 *   for(const auto & w : window.iterateFromTime(1.6))
 *   {
 *     mc_rtc::log::info(
 *       "Preview element with global index: {}, time: {:.3f}\n"
 *       "                     local  index: {}, time: {:.3f}\n"
 *     , w.index(), w.time()
 *     , w.localIndex(), w.localTime()
 *     );
 *     // Global time starts at 1.6s, while local time is between [0, window.duration()]
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
    startAtTime(startTime);
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
    return iterator{*this, endIndex() + 1};
  }

  const_iterator cbegin() noexcept
  {
    return const_iterator{*this, startIndex()};
  }
  const_iterator cend() noexcept
  {
    return const_iterator{*this, endIndex() + 1};
  }

  /**
   * @brief Iterate over a copy of the preview window
   *
   * The preview window constains only the window parameters and is cheap to
   * copy.
   *
   * @param startTime Time from which to iterate
   *
   * @return A copy of the preview window starting at time startTime
   */
  CenteredPreviewWindow iterateFromTime(double startTime) const noexcept
  {
    CenteredPreviewWindow w = *this;
    w.startAtTime(startTime);
    return w;
  }

  /**
   * @brief Iterate over a copy of the preview window
   *
   * The preview window constains only the window parameters and is cheap to
   * copy.
   *
   * @param startIndex Index from which to iterate
   *
   * @return A copy of the preview window starting at index startIndex
   */
  CenteredPreviewWindow iterateFromIndex(unsigned startIndex) const noexcept
  {
    CenteredPreviewWindow w = *this;
    w.startAtIndex(startIndex);
    return w;
  }

  /**
   * @brief Generate a copy of the current preview window
   */
  CenteredPreviewWindow copy() const noexcept
  {
    return *this;
  }
  // @}

  /**
   * @brief Start the preview window at the provided time
   *
   * @param t Time at which the window starts
   */
  void startAtTime(double t) noexcept
  {
    start_ = indexFromTime(t);
  }

  /**
   * @brief Start the preview window at the provided index
   */
  void startAtIndex(unsigned index) noexcept
  {
    start_ = index;
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
    return endIndex() * dt_;
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

  /**
   * @brief Timestep
   */
  double dt() const noexcept
  {
    return dt_;
  }

  /**
   * @brief Number of future or past elements.
   *
   * The preview window is of size `2*halfSize_+1`
   */
  unsigned halfSize() const noexcept
  {
    return halfSize_;
  }

  /**
   * @brief Duration corresponding to halfSize()
   */
  double halfDuration() const noexcept
  {
    return halfSize_ * dt_;
  }

  /**
   * @brief Full size of the preview window
   */
  unsigned size() const noexcept
  {
    return fullSize_;
  }

  /**
   * @brief Full duration of the preview window
   */
  double duration() const noexcept
  {
    return (size() - 1) * dt_;
  }

protected:
  unsigned halfSize_ = 0; ///< Number of future or past element
  unsigned fullSize_ = 0; ///< 2*halfSize_+1
  unsigned start_ = 0; ///< Index at which the window starts
  double dt_ = 0.005; ///< Timestep
};

} // namespace mc_planning
