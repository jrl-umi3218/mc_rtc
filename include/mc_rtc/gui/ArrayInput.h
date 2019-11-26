/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc
{

namespace gui
{

/** ArrayInput should display an array (\see ArrayLabel) and
 * allow editing
 *
 * \tparam GetT Returns an array-like object
 *
 * \tparam SetT Called when the client inputs a new value for
 * the array
 *
 */
template<typename GetT, typename SetT>
struct ArrayInputImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::ArrayInput;

  /** Constructor (no labels) */
  ArrayInputImpl(const std::string & name, GetT get_fn, SetT set_fn) : ArrayInputImpl(name, {}, get_fn, set_fn) {}

  /** Constructor with labels per-dimension */
  ArrayInputImpl(const std::string & name, const std::vector<std::string> & labels, GetT get_fn, SetT set_fn)
  : CommonInputImpl<GetT, SetT>::CommonInputImpl(name, get_fn, set_fn), labels_(labels)
  {
  }

  static constexpr size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & writer)
  {
    CommonInputImpl<GetT, SetT>::write(writer);
    writer.write(labels_);
  }

  /** Invalid element */
  ArrayInputImpl() {}

private:
  std::vector<std::string> labels_;
};

/** Helper function to create an ArrayInput element (no labels) */
template<typename GetT, typename SetT>
ArrayInputImpl<GetT, SetT> ArrayInput(const std::string & name, GetT get_fn, SetT set_fn)
{
  return ArrayInputImpl<GetT, SetT>(name, get_fn, set_fn);
}

/** Helper function to create an ArrayInput element (with labels) */
template<typename GetT, typename SetT>
ArrayInputImpl<GetT, SetT> ArrayInput(const std::string & name,
                                      const std::vector<std::string> & labels,
                                      GetT get_fn,
                                      SetT set_fn)
{
  return ArrayInputImpl<GetT, SetT>(name, labels, get_fn, set_fn);
}

} // namespace gui

} // namespace mc_rtc
