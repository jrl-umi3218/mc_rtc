/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc
{

namespace gui
{

/** Create a user-defined form
 *
 * A form is composed of 1 or more elements (FormInput) that can be required for the form completion (this must be
 * checked on client side)
 *
 * \tparam Callback Will be called when the form is completed on client side
 *
 */
template<typename Callback>
struct FormImpl : public CallbackElement<Element, Callback>
{
  static constexpr auto type = Elements::Form;

  template<typename... Args>
  FormImpl(const std::string & name, Callback cb, Args &&... args) : CallbackElement<Element, Callback>(name, cb)
  {
    mc_rtc::MessagePackBuilder builder(data_);
    count_ = sizeof...(Args);
    write_elements(builder, std::forward<Args>(args)...);
    data_size_ = builder.finish();
  }

  template<typename T>
  void addElement(T && element)
  {
    count_ += 1;
    std::vector<char> data = data_;
    mc_rtc::MessagePackBuilder builder(data_);
    builder.write_object(data.data(), data_size_);
    builder.start_array(element.write_size());
    element.write(builder);
    builder.finish_array();
    data_size_ = builder.finish();
  }

  static constexpr size_t write_size()
  {
    return CallbackElement<Element, Callback>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CallbackElement<Element, Callback>::write(builder);
    builder.start_array(count_);
    builder.write_object(data_.data(), data_size_);
    builder.finish_array();
  }

  /** Invalid element */
  FormImpl() {}

private:
  template<typename... Args>
  void write_elements(mc_rtc::MessagePackBuilder &, Args &&...)
  {
  }

  template<typename Arg, typename... Args>
  void write_elements(mc_rtc::MessagePackBuilder & builder, Arg && element, Args &&... args)
  {
    builder.start_array(element.write_size());
    element.write(builder);
    builder.finish_array();
    write_elements(builder, std::forward<Args>(args)...);
  }

  size_t count_;
  std::vector<char> data_;
  size_t data_size_;
};

template<typename Derived, Elements element>
struct FormElement
{
  static constexpr auto type = element;

  static constexpr size_t write_size()
  {
    return 3 + Derived::write_size_();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(name_);
    builder.write(static_cast<typename std::underlying_type<Elements>::type>(type));
    builder.write(required_);
    static_cast<Derived &>(*this).write_(builder);
  }

  /** Invalid element */
  FormElement() {}

protected:
  FormElement(const std::string & name, bool required) : name_(name), required_(required) {}

  std::string name_;
  bool required_;
};

template<typename T, Elements element>
struct FormDataInput : public FormElement<FormDataInput<T, element>, element>
{
  FormDataInput(const std::string & name, bool required, const T & def)
  : FormElement<FormDataInput<T, element>, element>(name, required), def_(def), has_def_(true)
  {
  }

  FormDataInput(const std::string & name, bool required) : FormDataInput<T, element>(name, required, {})
  {
    has_def_ = false;
  }

  static constexpr size_t write_size_()
  {
    return 2;
  }

  void write_(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(def_);
    builder.write(has_def_);
  }

  /** Invalid element */
  FormDataInput() {}

private:
  T def_;
  bool has_def_;
};

using FormCheckbox = FormDataInput<bool, Elements::Checkbox>;
using FormIntegerInput = FormDataInput<int, Elements::IntegerInput>;
using FormNumberInput = FormDataInput<double, Elements::NumberInput>;
using FormStringInput = FormDataInput<std::string, Elements::StringInput>;

template<typename T>
struct FormArrayInput : public FormElement<FormArrayInput<T>, Elements::ArrayInput>
{
  FormArrayInput(const std::string & name, bool required, const T & def, bool fixed_size = true)
  : FormElement<FormArrayInput, Elements::ArrayInput>(name, required), def_(def), fixed_size_(fixed_size),
    has_def_(true)
  {
  }

  FormArrayInput(const std::string & name, bool required, bool fixed_size = false)
  : FormArrayInput<T>(name, required, {}, fixed_size)
  {
    has_def_ = false;
  }

  static constexpr size_t write_size_()
  {
    return 3;
  }

  void write_(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(def_);
    builder.write(fixed_size_);
    builder.write(has_def_);
  }

  /** Invalid element */
  FormArrayInput() {}

private:
  T def_;
  bool fixed_size_;
  bool has_def_;
};

struct FormComboInput : public FormElement<FormComboInput, Elements::ComboInput>
{
  inline FormComboInput(const std::string & name,
                        bool required,
                        const std::vector<std::string> & values,
                        bool send_index = false,
                        int def = -1)
  : FormElement<FormComboInput, Elements::ComboInput>(name, required), values_(values), send_index_(send_index),
    def_(def)
  {
  }

  static constexpr size_t write_size_()
  {
    return 3;
  }

  inline void write_(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(values_);
    builder.write(send_index_);
    builder.write(def_);
  }

  /** Invalid element */
  inline FormComboInput() {}

private:
  std::vector<std::string> values_;
  bool send_index_ = false;
  int def_;
};

struct FormDataComboInput : public FormElement<FormDataComboInput, Elements::DataComboInput>
{
  static constexpr auto type = Elements::DataComboInput;

  inline FormDataComboInput(const std::string & name,
                            bool required,
                            const std::vector<std::string> & ref,
                            bool send_index = false)
  : FormElement<FormDataComboInput, Elements::DataComboInput>(name, required), ref_(ref), send_index_(send_index)
  {
  }

  static constexpr size_t write_size_()
  {
    return 2;
  }

  inline void write_(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(ref_);
    builder.write(send_index_);
  }

  /** Invalid element */
  inline FormDataComboInput() {}

private:
  std::vector<std::string> ref_;
  bool send_index_;
};

/** Helper to create a Form element */
template<typename Callback, typename... Args>
FormImpl<Callback> Form(const std::string & name, Callback cb, Args &&... args)
{
  return FormImpl<Callback>(name, cb, std::forward<Args>(args)...);
}

} // namespace gui

} // namespace mc_rtc
