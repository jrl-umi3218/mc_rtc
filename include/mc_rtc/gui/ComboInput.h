/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/elements.h>

namespace mc_rtc
{

namespace gui
{

/** ComboInput should display a combo box with the set of choices provided by values
 *
 * \tparam GetT Should return the current choice
 *
 * \tparam SetT Should accept the choice made by the user
 */
template<typename GetT, typename SetT>
struct ComboInputImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::ComboInput;

  ComboInputImpl(const std::string & name, const std::vector<std::string> & values, GetT get_fn, SetT set_fn)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn), values_(values)
  {
    static_assert(details::CheckReturnType<GetT, std::string>::value,
                  "ComboInput element getter callback should return an std::string");
  }

  static constexpr size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & writer)
  {
    CommonInputImpl<GetT, SetT>::write(writer);
    writer.write(values_);
  }

  /** Invalid element */
  ComboInputImpl() {}

private:
  std::vector<std::string> values_;
};

/** Helper function to create a ComboInputImpl */
template<typename GetT, typename SetT>
ComboInputImpl<GetT, SetT> ComboInput(const std::string & name,
                                      const std::vector<std::string> & values,
                                      GetT get_fn,
                                      SetT set_fn)
{
  return ComboInputImpl<GetT, SetT>(name, values, get_fn, set_fn);
}

} // namespace gui

} // namespace mc_rtc
