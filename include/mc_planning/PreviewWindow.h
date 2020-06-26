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
struct PreviewWindowView;

namespace internal
{

/**
 * @brief Provides a stronger type to disallow implicit conversions
 *
 * See https://foonathan.net/2016/10/strong-typedefs/
 */
template<class Tag, typename T>
class strong_typedef
{
public:
  strong_typedef() : value_() {}

  explicit strong_typedef(const T & value) : value_(value) {}

  explicit strong_typedef(T && value) noexcept(std::is_nothrow_move_constructible<T>::value) : value_(std::move(value))
  {
  }

  // Allow implicit conversion to T
  operator T &() noexcept
  {
    return value_;
  }

  // Allow implicit conversion to T const
  operator const T &() const noexcept
  {
    return value_;
  }

  friend void swap(strong_typedef & a, strong_typedef & b) noexcept
  {
    using std::swap;
    swap(static_cast<T &>(a), static_cast<T &>(b));
  }

private:
  T value_;
};

} // namespace internal

struct Time : internal::strong_typedef<Time, double>
{
  using strong_typedef::strong_typedef; // make constructors available
};
struct Index : internal::strong_typedef<Time, unsigned>
{
  using strong_typedef::strong_typedef; // make constructors available
};

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
  explicit PreviewElement(const PreviewWindowView & window, Index index) noexcept : window_(window), index_(index) {}

  explicit PreviewElement(const PreviewWindowView & window, unsigned index) noexcept : window_(window), index_(index) {}

  /**
   * @brief Returns the global index starting at `window.startIndex()`
   */
  Index index() const noexcept;
  /**
   * @brief Returns the time corresponding to index()
   */
  Time time() const noexcept;

  /**
   * @brief Returns the index local to the preview window
   *
   * @return Index between \f$ [0, 2*N] \f$ where `N` is the preview size
   */
  Index localIndex() const noexcept;

  /**
   * @brief Returns the time corresponding to localIndex()
   */
  Time localTime() const noexcept;

protected:
  const PreviewWindowView & window_; ///< Window being iterated over
  Index index_{0u}; ///< Global index
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
  /**
   * @brief Constructs a window of duration `2*halfTime` with the current
   * element at the center.
   *
   * @param halfTime Half-duration of the window
   * @param dt Timesep
   * @param startTime Time at which the window starts
   */
  explicit CenteredPreviewWindow(Time halfTime, Time dt) noexcept
  : halfSize_(static_cast<unsigned>(std::lround(halfTime / dt))), fullSize_(2 * halfSize_ + 1), dt_(dt)
  {
  }

  explicit CenteredPreviewWindow(double halfTime, double dt) noexcept : CenteredPreviewWindow(Time{halfTime}, Time{dt})
  {
  }

  /**
   * @brief Timestep
   */
  Time dt() const noexcept
  {
    return dt_;
  }

  /**
   * @brief Number of future or past elements.
   *
   * The preview window is of size `2*halfSize_+1`
   */
  Index halfSize() const noexcept
  {
    return halfSize_;
  }

  /**
   * @brief Duration corresponding to halfSize()
   */
  Time halfDuration() const noexcept
  {
    return Time{halfSize_ * dt_};
  }

  /**
   * @brief Full size of the preview window
   */
  Index size() const noexcept
  {
    return fullSize_;
  }

  /**
   * @brief Full duration of the preview window
   */
  Time duration() const noexcept
  {
    return Time{(size() - 1) * dt_};
  }

  /**
   * @brief Converts between time and index
   *
   * @param t Time
   *
   * @return Index corresponding to the provided time
   */
  Index index(Time t) const noexcept
  {
    return Index{static_cast<unsigned>(std::lround(t / dt_))};
  }

  /**
   * @brief Converts between index and time
   *
   * @param index Index
   *
   * @return Time corresponding to the provided index
   */
  Time time(Index index) const noexcept
  {
    return Time{index * dt_};
  }

  Index center()
  {
    return halfSize_;
  }

  PreviewWindowView all(Index index) const noexcept;
  PreviewWindowView all(Time startTime) const noexcept;
  // PreviewWindowView past(Index startIndex) const noexcept;
  // PreviewWindowView future(Index startIndex) const noexcept;

protected:
  Index halfSize_{0u}; ///< Number of future or past element
  Index fullSize_{0u}; ///< 2*halfSize_+1
  Time dt_{0.005}; ///< Timestep
};

struct PreviewWindowView
{
  using iterator = RangeElementIterator<PreviewElement, PreviewWindowView, false>;
  using const_iterator = RangeElementIterator<PreviewElement, PreviewWindowView, true>;

  explicit PreviewWindowView(const CenteredPreviewWindow & window,
                             Index absoluteStart,
                             Index localStart,
                             Index localEnd)
  : window_(window), absoluteStart_(absoluteStart), localStart_(localStart), localEnd_(localEnd)
  {
  }

  Index index(Time t) const noexcept
  {
    return window_.index(t);
  }

  /**
   * @brief Converts between index and time
   *
   * @param index Index
   *
   * @return Time corresponding to the provided index
   */
  Time time(Index index) const noexcept
  {
    return window_.time(index);
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
  // @}

  /**
   * @brief Time at the start of the preview window
   */
  inline Time startTime() const noexcept
  {
    return Time{absoluteStart_ * window_.dt()};
  }

  /**
   * @brief Index of the start of the preview window
   */
  Index startIndex() const noexcept
  {
    return absoluteStart_;
  }

  // /**
  //  * @brief Time at the center of the preview window
  //  */
  // Time currentTime() const noexcept
  // {
  //   return (start_ + window_.halfSize()) * window_.dt();
  // }

  // /**
  //  * @brief Index of the center of the preview window
  //  */
  // Index currentIndex() const noexcept
  // {
  //   return start_ + window_.halfSize();
  // }

  /**
   * @brief Index of the end of the preview window
   */
  Time endTime() const noexcept
  {
    return Time{endIndex() * window_.dt()};
  }

  /**
   * @brief Index of the end of the preview window
   */
  Index endIndex() const noexcept
  {
    return Index{absoluteStart_ + window_.size() - 1};
  }

protected:
  const CenteredPreviewWindow & window_; ///< Preview window parameters
  Index absoluteStart_{0}; ///< Index at which the view starts
  Index localStart_{0}; ///< Index within the preview window where it starts
  Index localEnd_{0}; ///< Index past end of the view
};

} // namespace mc_planning
