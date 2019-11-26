/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc
{

namespace gui
{

/** DataComboInput should behave like ComboInput but the data source is stored
 * in the GUI data store
 *
 * \tparam GetT Should return the current choice
 *
 * \tparam SetT Should accept the choice made by the user
 */
template<typename GetT, typename SetT>
struct DataComboInputImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::DataComboInput;

  DataComboInputImpl(const std::string & name, const std::vector<std::string> & data_ref, GetT get_fn, SetT set_fn)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn), data_ref_(data_ref)
  {
  }

  static constexpr size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 1;
  }

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

/** Helper function to build a DataComboInputImpl */
template<typename GetT, typename SetT>
DataComboInputImpl<GetT, SetT> DataComboInput(const std::string & name,
                                              const std::vector<std::string> & values,
                                              GetT get_fn,
                                              SetT set_fn)
{
  return DataComboInputImpl<GetT, SetT>(name, values, get_fn, set_fn);
}

} // namespace gui

} // namespace mc_rtc
