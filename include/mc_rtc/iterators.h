/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <memory>
#include <vector>

namespace mc_rtc
{

/** An iterator over std::vector<std::shared_ptr<T>> but it exposes T references/pointers instead of the shared pointer
 */
template<typename T>
struct SharedPtrVectorIterator : public std::vector<std::shared_ptr<T>>::iterator
{
  using iterator_t = typename std::vector<std::shared_ptr<T>>::iterator;

  SharedPtrVectorIterator(iterator_t it) noexcept : iterator_t(it) {}

  using pointer = T *;
  using reference = T &;

  reference operator*() const noexcept
  {
    return **static_cast<const iterator_t &>(*this);
  }

  pointer operator->() const noexcept
  {
    return static_cast<const iterator_t &>(*this)->get();
  }
};

/** A const_iterator over std::vector<std::shared_ptr<T>> but it exposes const T references/pointers instead of the
 * shared pointer
 */
template<typename T>
struct SharedPtrVectorConstIterator : public std::vector<std::shared_ptr<T>>::const_iterator
{
  using iterator_t = typename std::vector<std::shared_ptr<T>>::const_iterator;

  SharedPtrVectorConstIterator(iterator_t it) : iterator_t(it) {}

  using pointer = const T *;
  using reference = const T &;

  reference operator*() const noexcept
  {
    return **static_cast<const iterator_t &>(*this);
  }

  pointer operator->() const noexcept
  {
    return static_cast<const iterator_t &>(*this)->get();
  }
};

} // namespace mc_rtc
