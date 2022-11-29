/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <memory>

/** Helpers for unique_ptr<void, void (*)(void *)> */

namespace mc_rtc
{

using void_ptr = std::unique_ptr<void, void (*)(void *)>;

template<typename T, typename... Args>
void_ptr make_void_ptr(Args &&... args)
{
  return {new T(std::forward<Args>(args)...), [](void * ptr) { delete static_cast<T *>(ptr); }};
}

} // namespace mc_rtc
