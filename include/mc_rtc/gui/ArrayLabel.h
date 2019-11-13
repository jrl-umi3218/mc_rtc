/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/Label.h>

namespace mc_rtc
{

namespace gui
{

/** ArrayLabel should display data in an array form
 *
 * Labels can be provided optionally
 *
 * \tparam GetT should return anything that can be serialized as an array
 *
 * \see LabelImpl to display simple data
 */
template<typename GetT>
struct ArrayLabelImpl : public LabelImpl<GetT>
{
  static constexpr auto type = Elements::ArrayLabel;

  ArrayLabelImpl(const std::string & name, GetT get_fn) : LabelImpl<GetT>(name, get_fn) {}

  ArrayLabelImpl(const std::string & name, const std::vector<std::string> & labels, GetT get_fn)
  : LabelImpl<GetT>(name, get_fn), labels_(labels)
  {
  }

  /** Invalid element */
  ArrayLabelImpl() {}

  static constexpr size_t write_size()
  {
    return LabelImpl<GetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & writer)
  {
    LabelImpl<GetT>::write(writer);
    writer.write(labels_);
  }

private:
  std::vector<std::string> labels_;
};

/** Helper function to build an ArrayLabelImpl (no labels) */
template<typename GetT>
ArrayLabelImpl<GetT> ArrayLabel(const std::string & name, GetT get_fn)
{
  return ArrayLabelImpl<GetT>(name, get_fn);
}

/** Helper function to build an ArrayLabelImpl (with labels) */
template<typename GetT>
ArrayLabelImpl<GetT> ArrayLabel(const std::string & name, const std::vector<std::string> & labels, GetT get_fn)
{
  return ArrayLabelImpl<GetT>(name, labels, get_fn);
}

} // namespace gui

} // namespace mc_rtc
