/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/Label.h>

namespace mc_rtc::gui
{

namespace details
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

  static constexpr size_t write_size() { return LabelImpl<GetT>::write_size() + 1; }

  void write(mc_rtc::MessagePackBuilder & writer)
  {
    LabelImpl<GetT>::write(writer);
    writer.write(labels_);
  }

private:
  std::vector<std::string> labels_;
};

} // namespace details

/** Helper function to build an ArrayLabelImpl (no labels) */
template<typename GetT, std::enable_if_t<std::is_invocable_v<GetT>, int> = 0>
auto ArrayLabel(const std::string & name, GetT get_fn)
{
  return details::ArrayLabelImpl(name, get_fn);
}

/** Helper function to build an ArrayLabelImpl (with labels) */
template<typename GetT, std::enable_if_t<std::is_invocable_v<GetT>, int> = 0>
auto ArrayLabel(const std::string & name, const std::vector<std::string> & labels, GetT get_fn)
{
  return details::ArrayLabelImpl(name, labels, get_fn);
}

/** Helper function to build an ArrayLabelImpl from a variable.
 *
 * Labels are automatically added for certain types, see \ref details::Labels<T>
 */
template<typename T, std::enable_if_t<!std::is_invocable_v<T>, int> = 0>
auto ArrayLabel(const std::string & name, const std::vector<std::string> & labels, T && value)
{
  return ArrayLabel(name, labels, details::read(std::forward<T>(value)));
}

/** Helper function to build an ArrayLabelImpl from a variable.
 *
 * Labels are automatically added for certain types, see \ref details::Labels<T>
 */
template<typename T, std::enable_if_t<!std::is_invocable_v<T>, int> = 0>
auto ArrayLabel(const std::string & name, T && value)
{
  using Labels = details::Labels<std::decay_t<T>>;
  auto callback = details::read(std::forward<T>(value));
  if constexpr(Labels::has_labels) { return ArrayLabel(name, Labels::labels, callback); }
  else
  {
    return details::ArrayLabelImpl(name, callback);
  }
}

/** Creates a label for RPY angles
 *
 * Defaults to degrees inputs, can be changed via the \tparam DegreesInput template parameter
 */
template<bool Degrees = true, typename T>
auto RPYLabel(const std::string & name, T && value)
{
  return ArrayLabel(name, details::RPYLabels<Degrees>::labels, details::read_rpy<Degrees>(std::forward<T>(value)));
}

} // namespace mc_rtc::gui
