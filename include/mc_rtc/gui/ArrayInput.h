/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

#include <mc_rbdyn/rpy_utils.h>

namespace mc_rtc::gui
{

namespace details
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

  static constexpr size_t write_size() { return CommonInputImpl<GetT, SetT>::write_size() + 1; }

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

} // namespace details

/** Helper function to create an ArrayInput element (no labels) */
template<typename GetT, typename SetT>
auto ArrayInput(const std::string & name, GetT get_fn, SetT set_fn)
{
  return details::ArrayInputImpl(name, get_fn, set_fn);
}

/** Helper function to create an ArrayInput element (with labels) */
template<typename GetT, typename SetT>
auto ArrayInput(const std::string & name, const std::vector<std::string> & labels, GetT get_fn, SetT set_fn)
{
  return details::ArrayInputImpl(name, labels, get_fn, set_fn);
}

/** Helper function to build an ArrayInput from a variable */
template<typename T>
auto ArrayInput(const std::string & name, const std::vector<std::string> & labels, T & value)
{
  return details::ArrayInputImpl(name, labels, details::read(value), details::write(value));
}

/** Helper function to build an ArrayInput from a variable
 *
 * Labels are automatically added for certain types, see \ref details::Labels<T>
 */
template<typename T>
auto ArrayInput(const std::string & name, T & value)
{
  using Labels = details::Labels<std::decay_t<T>>;
  auto read = details::read(value);
  auto write = details::write(value);
  if constexpr(Labels::has_labels) { return ArrayInput(name, Labels::labels, read, write); }
  else
  {
    return ArrayInput(name, read, write);
  }
}

/** Creates an input for a rotation using RPY angles
 *
 * Defaults to degrees inputs, can be changed via the \tparam Degrees template parameter
 */
template<bool Degrees = true, typename T>
auto RPYInput(const std::string & name, T & value)
{
  return ArrayInput(name, details::RPYLabels<Degrees>::labels, details::read_rpy<Degrees>(value),
                    [&value](const Eigen::Vector3d & rpy)
                    {
                      if constexpr(Degrees) { value = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180.); }
                      else
                      {
                        value = mc_rbdyn::rpyToMat(rpy);
                      }
                    });
}

} // namespace mc_rtc::gui
