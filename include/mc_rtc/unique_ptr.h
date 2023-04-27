/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <memory>

namespace mc_rtc
{

/** Wrapper arround unique_ptr<T>
 *
 * Forbids reset() without argument
 *
 * This is mostly meant to avoid transition issues such as 4137311efc5615ec0807b7749a036ab7ecb91086
 */
template<class T, class Deleter = std::default_delete<T>>
class unique_ptr : public std::unique_ptr<T, Deleter>
{
public:
  using std::unique_ptr<T, Deleter>::unique_ptr;

  void reset(typename std::unique_ptr<T, Deleter>::pointer ptr) { std::unique_ptr<T, Deleter>::reset(ptr); }
};

} // namespace mc_rtc
