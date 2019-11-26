/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc
{

namespace gui
{

/** Should display a form using the data provided in the configured schema folder
 *
 * The schema folder should be relative to mc_rtc install path
 *
 * \tparam Callback Called with form data when the user completes the form
 *
 */
template<typename Callback>
struct SchemaImpl : public CallbackElement<Element, Callback>
{
  static constexpr auto type = Elements::Schema;

  SchemaImpl(const std::string & name, const std::string & schema, Callback cb)
  : CallbackElement<Element, Callback>(name, cb), schema_(schema)
  {
  }

  static constexpr size_t write_size()
  {
    return CallbackElement<Element, Callback>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CallbackElement<Element, Callback>::write(builder);
    builder.write(schema_);
  }

private:
  std::string schema_;
};

/** Helper function to create a Schema element */
template<typename Callback>
SchemaImpl<Callback> Schema(const std::string & name, const std::string & schema, Callback cb)
{
  return SchemaImpl<Callback>(name, schema, cb);
}

} // namespace gui

} // namespace mc_rtc
