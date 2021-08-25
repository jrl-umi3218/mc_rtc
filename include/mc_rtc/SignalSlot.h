/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <cassert>
#include <functional>
#include <vector>

namespace mc_rtc
{

template<typename... ArgsT>
struct Signal;

template<typename... ArgsT>
struct Slot;

/** A signal class
 *
 * Provides a way to \ref connect to the signal, i.e. register a callback that
 * will be called whenever the signal is raised via the \ref signal method.
 *
 *
 * You can use this mechanism in your own code (controller, state, interface,
 * observer...) however you must be aware of the following assumptions that are
 * true to all usage within mc_rtc:
 * - the signal emitter outlives all subscribers
 * - the signal is only emitted on the main execution thread
 * - the signal is handled as soon as it is emitted
 *
 * \tparam ArgsT Arguments of the signal callback
 *
 */
template<typename... ArgsT>
struct Signal
{
  using SignalT = Signal<ArgsT...>;
  using SlotT = Slot<ArgsT...>;
  friend SlotT;
  using CallbackT = std::function<void(ArgsT...)>;

  Signal() = default;

  /** Connect a callback to the signal and returns a connected slot
   *
   * \param callback Any callback that will match \tparam ArgsT
   */
  template<typename Callable>
  SlotT connect(Callable && callback)
  {
    SlotT out{this, next_slot_};
    if(next_slot_ < callbacks_.size())
    {
      callbacks_[next_slot_] = callback;
    }
    else
    {
      callbacks_.push_back(callback);
    }
    next_slot_++;
    while(next_slot_ < callbacks_.size() && callbacks_[next_slot_])
    {
      next_slot_++;
    }
    return out;
  }

  /** Trigger a signal with the provided arguments
   *
   * \params args Arguments passed to subscribers
   *
   */
  void signal(ArgsT... args)
  {
    for(auto & c : callbacks_)
    {
      if(c)
      {
        c(args...);
      }
    }
  }

  /** An helper class that expose only the connect function
   *
   * This is useful to allow connections without emissions
   */
  struct Proxy
  {
    Proxy(SignalT & self) : self_(self) {}

    template<typename Callable>
    SlotT connect(Callable && callback)
    {
      return self_.connect(std::forward<Callable>(callback));
    }

  private:
    SignalT & self_;
  };

protected:
  void disconnect(size_t idx)
  {
    assert(idx < callbacks_.size());
    callbacks_[idx] = {};
    next_slot_ = std::min(idx, next_slot_);
  }
  std::vector<CallbackT> callbacks_;
  size_t next_slot_ = 0;
};

/** A slot class
 *
 * \tparam ArgsT Arguments of the signal callback
 *
 */
template<typename... ArgsT>
struct Slot
{
  using SignalT = Signal<ArgsT...>;
  friend SignalT;

  Slot() = default;
  Slot(const Slot &) = delete;
  Slot(Slot && rhs)
  {
    signal_ = rhs.signal_;
    idx_ = rhs.idx_;
    rhs.signal_ = nullptr;
  }
  Slot & operator=(const Slot &) = delete;
  Slot & operator=(Slot && rhs)
  {
    if(&rhs == this)
    {
      return *this;
    }
    signal_ = rhs.signal_;
    idx_ = rhs.idx_;
    rhs.signal_ = nullptr;
    return *this;
  }

  ~Slot()
  {
    disconnect();
  }

  void disconnect()
  {
    if(signal_)
    {
      signal_->disconnect(idx_);
      signal_ = nullptr;
    }
  }

protected:
  Slot(SignalT * sig, size_t idx) : signal_(sig), idx_(idx) {}

private:
  SignalT * signal_ = nullptr;
  size_t idx_ = 0;
};

} // namespace mc_rtc
