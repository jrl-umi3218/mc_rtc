/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc::gui
{

namespace details
{

/** DataComboInput should behave like ComboInput but the data source is stored
 * in the GUI data store
 *
 * \tparam GetT Should return the current choice
 *
 * \tparam SetT Should accept the choice made by the user
 */
template<typename GetT, typename SetT, typename LabelsContainerT = std::vector<std::string>>
struct DataComboInputImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::DataComboInput;

  DataComboInputImpl(const std::string & name, const LabelsContainerT & data_ref, GetT get_fn, SetT set_fn)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn), data_ref_(details::to_string_vector(data_ref))
  {
  }

  static constexpr size_t write_size() { return CommonInputImpl<GetT, SetT>::write_size() + 1; }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CommonInputImpl<GetT, SetT>::write(builder);
    builder.write(data_ref_);
  }

  /** Invalid element */
  DataComboInputImpl() {}

private:
  std::vector<std::string> data_ref_;
};

} // namespace details

/** Helper function to build a DataComboInputImpl */
template<typename GetT, typename SetT, typename LabelsContainerT = std::vector<std::string>>
auto DataComboInput(const std::string & name, const LabelsContainerT & values, GetT get_fn, SetT set_fn)
{
  return details::DataComboInputImpl(name, values, get_fn, set_fn);
}

/** Helper function to build a DataComboInputImpl from a variable */
template<typename LabelsContainerT = std::vector<std::string>>
inline auto DataComboInput(const std::string & name, const LabelsContainerT & values, std::string & value)
{
  return DataComboInput(name, values, details::read(value), details::write(value));
}

} // namespace mc_rtc::gui
