/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_planning/api.h>
#include <iterator>
#include <type_traits>

namespace mc_planning
{

/**
 * @brief Virtual iterator for iterating over a range of value
 * and accessing elements of the parent class
 *
 * \see CenteredPreviewWindow for a practical use
 */
template<typename RetT, typename ParentT, bool Const = false>
struct MC_PLANNING_DLLAPI RangeElementIterator
{
private:
  struct IncrementHolder
  {
    explicit IncrementHolder(unsigned value) noexcept : index_(value) {}
    unsigned operator*() noexcept
    {
      return index_;
    }

  private:
    unsigned index_;
  };

public:
  using value_type = RetT;
  /* deduce const qualifier from bool Const parameter */
  using reference = typename std::conditional<Const, RetT const &, RetT &>::type;
  using pointer = typename std::conditional<Const, RetT const *, RetT *>::type;
  using iterator_category = std::input_iterator_tag;
  using difference_type = void;

  explicit RangeElementIterator(const ParentT & window, unsigned value) noexcept : window_(window), index_(value) {}
  value_type operator*() const
  {
    return value_type(window_, index_);
  }
  bool operator==(const RangeElementIterator & other) const noexcept
  {
    return index_ == other.index_;
  }
  bool operator!=(const RangeElementIterator & other) const noexcept
  {
    return !(*this == other);
  }
  IncrementHolder operator++(int) noexcept
  {
    IncrementHolder ret(index_);
    ++*this;
    return ret;
  }
  RangeElementIterator & operator++() noexcept
  {
    ++index_;
    return *this;
  }

private:
  const ParentT & window_;
  unsigned index_;
};

} // namespace mc_planning
