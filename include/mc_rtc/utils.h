#pragma once

#include <mc_rtc/logging.h>

#include <atomic>

namespace
{

/** Simple lock-free thread-safe single-producer single-consumer circular
 * buffer
 *
 * Loosely based off the article: https://www.codeproject.com/Articles/43510/Lock-Free-Single-Producer-Single-Consumer-Circular
 *
 */
template<typename T, size_t Size>
struct CircularBuffer
{
public:
  enum { Capacity = Size + 1 };

  CircularBuffer() : tail_(0), head_(0)
  {
    if(!tail_.is_lock_free())
    {
      LOG_WARNING("Your platform does not support std::atomic_size_t as lock free operations")
    }
  }

  /** Returns false if the push failed (i.e. full buffer) */
  bool push(const T & item)
  {
    size_t tail = tail_;
    auto next_tail = increment(tail);
    if(next_tail != head_)
    {
      data_[tail] = item;
      tail_ = next_tail;
      return true;
    }
    return false;
  }

  /** Returns false if the pop failed (i.e. empty buffer) */
  bool pop(T & item)
  {
    const size_t head = head_;
    if(head == tail_)
    {
      return false;
    }
    item = data_[head];
    head_ = increment(head);
    return true;
  }

  /** Returns true if the buffer is empty */
  bool empty()
  {
    return head_ == tail_;
  }
private:
  size_t increment(size_t idx) const
  {
    return (idx + 1) % Capacity;
  }

  T data_[Capacity];
  std::atomic_size_t tail_;
  std::atomic_size_t head_;
};

}

