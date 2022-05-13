/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <memory>

/** Helpers for unique_ptr<void, void (*)(void *)> */

namespace mc_rtc
{

using void_ptr = std::unique_ptr<void, void (*)(void *)>;

/** Creates a new void_ptr holding an object of type T */
template<typename T, typename... Args>
void_ptr make_void_ptr(Args &&... args)
{
  return {new T(std::forward<Args>(args)...), [](void * ptr) { delete static_cast<T *>(ptr); }};
}

/** Transfer ownership of a unique_ptr<T> to a void_ptr */
template<typename T>
void_ptr make_void_ptr(std::unique_ptr<T> ptr)
{
  return {ptr.release(), [](void * ptr) { delete static_cast<T *>(ptr); }};
}

/** Helper to reduce boiler-plate code in tasks/constraints implementation */
template<typename T>
struct void_ptr_caster
{
  T * operator()(void_ptr & ptr)
  {
    return static_cast<T *>(ptr.get());
  }

  const T * operator()(const void_ptr & ptr)
  {
    return static_cast<const T *>(ptr.get());
  }
};

} // namespace mc_rtc
