/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/elements.h>

namespace mc_rtc::gui
{

namespace details
{

/** Store elements of a form */
struct FormElements
{
  template<typename... Args>
  FormElements(Args &&... args)
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
    using ElementT = typename std::decay<T>::type;
    if(ElementT::is_dynamic()) { addDynamicElement(std::forward<T>(element)); }
    else
    {
      std::vector<char> data = data_;
      mc_rtc::MessagePackBuilder builder(data_);
      builder.write_object(data.data(), data_size_);
      builder.start_array(element.write_size());
      element.write(builder);
      builder.finish_array();
      data_size_ = builder.finish();
    }
  }

  void write_impl(mc_rtc::MessagePackBuilder & builder)
  {
    builder.start_array(count_);
    for(const auto & el : dynamic_elements_) { el(builder); }
    builder.write_object(data_.data(), data_size_);
    for(size_t i = dynamic_elements_.size() + 1; i < count_; ++i) { builder.write_object(nullptr, 0); }
    builder.finish_array();
  }

protected:
  template<typename... Args>
  void write_elements(mc_rtc::MessagePackBuilder &, Args &&...)
  {
  }

  template<typename Arg, typename... Args>
  void write_elements(mc_rtc::MessagePackBuilder & builder, Arg && element, Args &&... args)
  {
    using ElementT = typename std::decay<Arg>::type;
    if(ElementT::is_dynamic()) { addDynamicElement(std::forward<Arg>(element)); }
    else
    {
      builder.start_array(element.write_size());
      element.write(builder);
      builder.finish_array();
    }
    write_elements(builder, std::forward<Args>(args)...);
  }

  template<typename T>
  void addDynamicElement(T && element)
  {
    auto callback = [element](mc_rtc::MessagePackBuilder & builder) mutable
    {
      builder.start_array(element.write_size());
      element.write(builder);
      builder.finish_array();
    };
    dynamic_elements_.push_back(callback);
  }

  size_t count_;
  std::vector<std::function<void(mc_rtc::MessagePackBuilder &)>> dynamic_elements_;
  std::vector<char> data_;
  size_t data_size_;
};

/** Create a user-defined form
 *
 * A form is composed of 1 or more elements (FormInput) that can be required for the form completion (this must be
 * checked on client side)
 *
 * \tparam Callback Will be called when the form is completed on client side
 *
 */
template<typename Callback>
struct FormImpl : public CallbackElement<Element, Callback>, FormElements
{
  static constexpr auto type = Elements::Form;

  template<typename... Args>
  FormImpl(const std::string & name, Callback cb, Args &&... args)
  : CallbackElement<Element, Callback>(name, cb), FormElements(std::forward<Args>(args)...)
  {
  }

  static constexpr size_t write_size() { return CallbackElement<Element, Callback>::write_size() + 1; }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CallbackElement<Element, Callback>::write(builder);
    FormElements::write_impl(builder);
  }

  /** Invalid element */
  FormImpl() {}
};

} // namespace details

template<typename Derived, Elements element>
struct FormElement
{
  static constexpr auto type = element;

  static constexpr size_t write_size() { return 3 + Derived::write_size_(); }

  static constexpr bool is_dynamic() { return false; }

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

namespace details
{

/** This helper class avoids forming reference to void arguments */
template<typename T, typename Callback>
struct CallbackOrValue
{
  static_assert(details::CheckReturnType<Callback, T>::value, "Callback should return the right type of value");

  Callback callback;

  void write(mc_rtc::MessagePackBuilder & builder) { builder.write(callback()); }
};

template<typename T>
struct CallbackOrValue<T, void>
{
  T value;

  void write(mc_rtc::MessagePackBuilder & builder) { builder.write(value); }
};

template<typename T, Elements element, typename DataCallback = void>
struct FormDataInput : public FormElement<FormDataInput<T, element, DataCallback>, element>
{
  FormDataInput(const std::string & name, bool required, CallbackOrValue<T, DataCallback> def)
  : FormElement<FormDataInput<T, element, DataCallback>, element>(name, required), def_(def), has_def_(true)
  {
  }

  FormDataInput(const std::string & name, bool required) : FormDataInput<T, element>(name, required, {})
  {
    has_def_ = false;
  }

  static constexpr size_t write_size_() { return 2; }

  static constexpr bool is_dynamic() { return !std::is_same<DataCallback, void>::value; }

  void write_(mc_rtc::MessagePackBuilder & builder)
  {
    def_.write(builder);
    builder.write(has_def_);
  }

  /** Invalid element */
  FormDataInput() {}

private:
  CallbackOrValue<T, DataCallback> def_;
  bool has_def_;
};

} // namespace details

#define MAKE_DATA_INPUT_HELPER(DATAT, ELEMENT, FNAME)                                                       \
  inline details::FormDataInput<DATAT, ELEMENT> FNAME(const std::string & name, bool required)              \
  {                                                                                                         \
    return {name, required};                                                                                \
  }                                                                                                         \
                                                                                                            \
  inline details::FormDataInput<DATAT, ELEMENT> FNAME(const std::string & name, bool required, DATAT value) \
  {                                                                                                         \
    return {name, required, details::CallbackOrValue<DATAT, void>{value}};                                  \
  }                                                                                                         \
                                                                                                            \
  template<typename Callback, typename std::enable_if<details::is_getter<Callback>(), int>::type = 0>       \
  inline details::FormDataInput<DATAT, ELEMENT, Callback> FNAME(const std::string & name, bool required,    \
                                                                Callback callback)                          \
  {                                                                                                         \
    return {name, required, details::CallbackOrValue<DATAT, Callback>{callback}};                           \
  }

MAKE_DATA_INPUT_HELPER(bool, Elements::Checkbox, FormCheckbox)
MAKE_DATA_INPUT_HELPER(int, Elements::IntegerInput, FormIntegerInput)
MAKE_DATA_INPUT_HELPER(double, Elements::NumberInput, FormNumberInput)
MAKE_DATA_INPUT_HELPER(std::string, Elements::StringInput, FormStringInput)
MAKE_DATA_INPUT_HELPER(Eigen::Vector3d, Elements::Point3D, FormPoint3DInput)
MAKE_DATA_INPUT_HELPER(sva::PTransformd, Elements::Rotation, FormRotationInput)
MAKE_DATA_INPUT_HELPER(sva::PTransformd, Elements::Transform, FormTransformInput)

#undef MAKE_DATA_INPUT_HELPER

namespace details
{

template<typename T, typename DataCallback = void>
struct FormArrayInput : public FormElement<FormArrayInput<T, DataCallback>, Elements::ArrayInput>
{
  FormArrayInput(const std::string & name, bool required, CallbackOrValue<T, DataCallback> def, bool fixed_size = true)
  : FormElement<FormArrayInput<T, DataCallback>, Elements::ArrayInput>(name, required), def_(def),
    fixed_size_(fixed_size), has_def_(true)
  {
  }

  FormArrayInput(const std::string & name, bool required, bool fixed_size = false)
  : FormArrayInput<T>(name, required, {}, fixed_size)
  {
    has_def_ = false;
  }

  static constexpr size_t write_size_() { return 3; }

  static constexpr bool is_dynamic() { return !std::is_same<DataCallback, void>::value; }

  void write_(mc_rtc::MessagePackBuilder & builder)
  {
    def_.write(builder);
    builder.write(fixed_size_);
    builder.write(has_def_);
  }

  /** Invalid element */
  FormArrayInput() {}

private:
  CallbackOrValue<T, DataCallback> def_;
  bool fixed_size_;
  bool has_def_;
};

} // namespace details

template<typename T>
details::FormArrayInput<T> FormArrayInput(const std::string & name, bool required, bool fixed_size = false)
{
  return {name, required, fixed_size};
}

template<typename T, typename std::enable_if<!details::is_getter<T>(), int>::type = 0>
inline details::FormArrayInput<T> FormArrayInput(const std::string & name,
                                                 bool required,
                                                 const T & value,
                                                 bool fixed_size = true)
{
  return {name, required, details::CallbackOrValue<T, void>{value}, fixed_size};
}

template<typename Callback, typename std::enable_if<details::is_getter<Callback>(), int>::type = 0>
inline details::FormArrayInput<details::ReturnTypeT<Callback>, Callback> FormArrayInput(const std::string & name,
                                                                                        bool required,
                                                                                        Callback callback,
                                                                                        bool fixed_size = true)
{
  using ReturnT = details::ReturnTypeT<Callback>;
  return {name, required, details::CallbackOrValue<ReturnT, Callback>{callback}, fixed_size};
}

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

  static constexpr size_t write_size_() { return 3; }

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
  inline FormDataComboInput(const std::string & name,
                            bool required,
                            const std::vector<std::string> & ref,
                            bool send_index = false)
  : FormElement<FormDataComboInput, Elements::DataComboInput>(name, required), ref_(ref), send_index_(send_index)
  {
  }

  static constexpr size_t write_size_() { return 2; }

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

/** Creates a form within another form
 *
 * If the object is required then it will be received in the callback and its own required elements are also contained
 * in the object
 */
struct FormObjectInput : public FormElement<FormObjectInput, Elements::Form>, details::FormElements
{
  template<typename... Args>
  FormObjectInput(const std::string & name, bool required, Args &&... args)
  : FormElement<FormObjectInput, Elements::Form>(name, required), FormElements(std::forward<Args>(args)...)
  {
  }

  static constexpr bool is_dynamic() { return true; }

  static constexpr size_t write_size_() { return 1; }

  void write_(mc_rtc::MessagePackBuilder & builder) { FormElements::write_impl(builder); }
};

/** Creates an inputs to build a generic array
 *
 * The element you pass to this element will be used to build the list, e.g.:
 * - FormCheckbox will output a list of booleans
 * - FormStringInput a list of strings
 * - FormPoint3DInput a list of 3D points
 *
 * The name of the element you pass to this function has no effect
 *
 * For more complex objects you can use a FormObjectInput or use FormObjectArrayInput which is more explicit
 */
struct FormGenericArrayInput : public FormElement<FormGenericArrayInput, Elements::GenericArray>,
                               private details::FormElements
{
  template<typename Element>
  FormGenericArrayInput(const std::string & name, bool required, Element && element)
  : FormElement<FormGenericArrayInput, Elements::GenericArray>(name, required),
    FormElements(std::forward<Element>(element))
  {
  }

  static constexpr bool is_dynamic() { return true; }

  static constexpr size_t write_size_() { return 1; }

  void write_(mc_rtc::MessagePackBuilder & builder) { FormElements::write_impl(builder); }
};

/** Creates an inputs to build an array of objects
 *
 * If the array is required the list is always sent even if it is empty otherwise it is only sent if it has at least one
 * item
 */
struct FormObjectArrayInput : public FormElement<FormObjectArrayInput, Elements::ObjectArray>, details::FormElements
{
  template<typename... Args>
  FormObjectArrayInput(const std::string & name, bool required, Args &&... args)
  : FormElement<FormObjectArrayInput, Elements::ObjectArray>(name, required), FormElements(std::forward<Args>(args)...)
  {
  }

  static constexpr bool is_dynamic() { return true; }

  static constexpr size_t write_size_() { return 1; }

  void write_(mc_rtc::MessagePackBuilder & builder) { FormElements::write_impl(builder); }
};

/** Creates a one-of selector
 *
 * Only one of the item provided to this input will be active, the item name will tell which was selected
 *
 * If required, one of the element must be selected
 */
struct FormOneOfInput : public FormElement<FormOneOfInput, Elements::OneOf>, details::FormElements
{
  template<typename... Args>
  FormOneOfInput(const std::string & name, bool required, Args &&... args)
  : FormElement<FormOneOfInput, Elements::OneOf>(name, required), FormElements(std::forward<Args>(args)...)
  {
  }

  static constexpr bool is_dynamic() { return true; }

  static constexpr size_t write_size_() { return 1; }

  void write_(mc_rtc::MessagePackBuilder & builder) { FormElements::write_impl(builder); }
};

/** Helper to create a Form element */
template<typename Callback, typename... Args>
auto Form(const std::string & name, Callback cb, Args &&... args)
{
  return details::FormImpl(name, cb, std::forward<Args>(args)...);
}

} // namespace mc_rtc::gui
