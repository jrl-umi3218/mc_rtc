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

  T operator()() const noexcept
  {
    return value_;
  }

  friend void swap(strong_typedef & a, strong_typedef & b) noexcept
  {
    using std::swap;
    swap(static_cast<T &>(a), static_cast<T &>(b));
  }

protected:
  T value_;
};

template<typename T, typename underlying_t>
struct strong_typedef_operators
{
  friend T operator+(const T & lhs, const T & rhs)
  {
    return T{lhs() + rhs()};
  }
  friend T operator-(const T & lhs, const T & rhs)
  {
    return T{lhs() - rhs()};
  }
  friend T operator+(const T & lhs, const underlying_t & rhs)
  {
    return T{lhs() + rhs};
  }
  friend T operator-(const T & lhs, const underlying_t & rhs)
  {
    return T{lhs() - rhs};
  }
};

} // namespace internal

struct Time : internal::strong_typedef<Time, double>, internal::strong_typedef_operators<Time, double>
{
  using strong_typedef::strong_typedef; // make constructors available
  Time operator+(const Time & t)
  {
    return Time{value_ + t};
  }
};
struct Index : internal::strong_typedef<Index, unsigned>, internal::strong_typedef_operators<Index, unsigned>
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
 * @brief Stores parameters related to a preview window
 *
 * This preview window is centered around the current element \f$ [-N, current, N] \f$ where `N` is the number of
 * elements before and after the current value. The full preview window is of size `2*N+1`. The timestep is used to
 * compute correspondances between discrete index of values stored in an (external) container and time.
 *
 * The @ref range_views functions can be used to obtain PreviewWindowView iterable ranges over past/future/all preview
 * elements for a window starting at any time.
 *
 * \see PreviewWindowView iterable ranges over a floating preview window
 *
 * For practical use, see:
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
    return Time{(size() - 1u) * dt_};
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

  Index center() const noexcept
  {
    return halfSize_;
  }

  /**
   * @defgoup range_views
   * Returns predefined views to the past/future of the preview window starting at the specified time/index
   * - all: all elements in the preview window
   * - past: all past elements, present excluded
   * - pastInclusive: all past elements, present included
   * - future: all future elements, present excluded
   * - futureInclusive: all future elements, present included
   * @{
   */
  /** All preview elements with a preview window starting at provided index */
  PreviewWindowView all(Index index) const noexcept;
  /** All preview elements with a preview window starting at provided time */
  PreviewWindowView all(Time startTime) const noexcept;
  /** Past preview elements (excluding present) with a preview window starting at provided index */
  PreviewWindowView past(Index index) const noexcept;
  /** Past preview elements (excluding present) with a preview window starting at provided time */
  PreviewWindowView past(Time startTime) const noexcept;
  /** Past preview elements (including present) with a preview window starting at provided index */
  PreviewWindowView pastInclusive(Index index) const noexcept;
  /** Past preview elements (including present) with a preview window starting at provided index */
  PreviewWindowView pastInclusive(Time startTime) const noexcept;
  /** Future preview elements (excluding present) with a preview window starting at provided index */
  PreviewWindowView future(Index index) const noexcept;
  /** Future preview elements (excluding present) with a preview window starting at provided time */
  PreviewWindowView future(Time startTime) const noexcept;
  /** Future preview elements (including present) with a preview window starting at provided index */
  PreviewWindowView futureInclusive(Index index) const noexcept;
  /** Future preview elements (including present) with a preview window starting at provided time */
  PreviewWindowView futureInclusive(Time startTime) const noexcept;
  /// @}

protected:
  Index halfSize_{0u}; ///< Number of future or past element
  Index fullSize_{0u}; ///< 2*halfSize_+1
  Time dt_{0.005}; ///< Timestep
};

/**
 * Provides an iterable view on a preview window for convenient safe access to
 * ranges of elements
 *
 * For example:
 * \code{.cpp}
 *   // Create an iterable preview window range with a window of time `2*1.6`,
 *   // timestep of 0.005
 *   CenteredPreviewWindow window(1.6, 0.005);
 *   mc_rtc::log::info("Start time:   {:.3f}, index: {}\n"
 *                     "End time:     {:.3f}, index: {}\n"
 *                     "Current time: {:.3f}, index: {}",
 *                     window.startTime(),    window.startIndex(),
 *                     window.endTime(),      window.endIndex(),
 *                     window.currentTime(),  window.currentIndex());
 *
 *   // Iterate over a view to the preview window starting at time t=0.5 and
 *   // containing all past elements in the preview window
 *   for(const auto & w : window.past(Time{0.5}))
 *   {
 *     mc_rtc::log::info("Preview element with index: {}, time: {:.3f}", w.index(), w.time());
 *     // Example: Access some external container using the index
 *     // auto value = container[w.index()];
 *   }
 *
 *   // Iterate over a view to the preview window starting at time t=1.6 and
 *   // containing all elements (past, present and future) in the preview window
 *   for(const auto & w : window.all(1.6))
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
 **/
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

  /** @brief Index corresponding to time t */
  Index index(Time t) const noexcept
  {
    return window_.index(t);
  }

  /** @brief Time corresponding to index */
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
    return iterator{*this, endIndex() + 1u};
  }

  const_iterator cbegin() noexcept
  {
    return const_iterator{*this, startIndex()};
  }
  const_iterator cend() noexcept
  {
    return const_iterator{*this, endIndex() + 1u};
  }
  // @}

  /**
   * @brief Time at the start of the preview window view
   */
  inline Time startTime() const noexcept
  {
    return Time{(absoluteStart_ + localStart_) * window_.dt()};
  }

  /**
   * @brief Index of the start of the preview window view
   */
  Index startIndex() const noexcept
  {
    return absoluteStart_ + localStart_;
  }

  /**
   * @brief Index of the end of the preview window view
   */
  Time endTime() const noexcept
  {
    return Time{endIndex() * window_.dt()};
  }

  /**
   * @brief Index of the end of the preview window view
   */
  Index endIndex() const noexcept
  {
    return absoluteStart_ + localEnd_;
  }

  /**
   * @brief Full size of the preview window view
   */
  Index size() const noexcept
  {
    return Index{localEnd_ - localStart_ + 1u};
  }

  /**
   * @brief Full duration of the preview window view
   */
  Time duration() const noexcept
  {
    return Time{(size() - 1u) * window_.dt()};
  }

  /**
   * @brief Index of the center of the preview window
   * @note May be outside of this view
   */
  Index nowIndex() const noexcept
  {
    return absoluteStart_ + window_.halfSize();
  }

  /**
   * @brief Time of the center of the preview window
   * @note May be outside of this view
   */
  Time nowTime() const noexcept
  {
    return Time{nowIndex() * window_.dt()};
  }

protected:
  const CenteredPreviewWindow & window_; ///< Preview window parameters
  Index absoluteStart_{
      0}; ///< Index at which the preview window starts. The view is defined relative to the start of the preivew window
  Index localStart_{0}; ///< Index within the preview window where it starts
  Index localEnd_{0}; ///< Index past end of the view
};

} // namespace mc_planning
