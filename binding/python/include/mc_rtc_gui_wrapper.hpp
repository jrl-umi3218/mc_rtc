/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/configuration_io.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/Core>
#include <functional>

using get_fn = std::function<mc_rtc::Configuration()>;
using set_fn = std::function<void(const mc_rtc::Configuration &)>;
using void_cb = std::function<void()>;

template<typename T, typename U>
get_fn make_getter(T fn, U arg)
{
  return [fn, arg] { return fn(arg); };
}

template<typename T, typename Cb, typename t>
set_fn make_setter(T fn, Cb cb, t type)
{
  return [fn, cb, type](const mc_rtc::Configuration & data) mutable {
    return fn(cb, type, const_cast<mc_rtc::Configuration &>(data));
  };
}

template<typename T, typename Cb>
void_cb make_void_cb(T fn, Cb cb)
{
  return [fn, cb]() mutable { return fn(cb); };
}

template<typename Callback>
using Form = mc_rtc::gui::FormImpl<Callback>;

template<typename Callback>
void add_form_checkbox(Form<Callback> & form, const std::string & name, bool required)
{
  form.addElement(mc_rtc::gui::FormCheckbox(name, required));
}

template<typename Callback>
void add_form_checkbox(Form<Callback> & form, const std::string & name, bool required, bool value)
{
  form.addElement(mc_rtc::gui::FormCheckbox(name, required, value));
}

template<typename Callback>
void add_form_integer(Form<Callback> & form, const std::string & name, bool required)
{
  form.addElement(mc_rtc::gui::FormIntegerInput(name, required));
}

template<typename Callback>
void add_form_integer(Form<Callback> & form, const std::string & name, bool required, int value)
{
  form.addElement(mc_rtc::gui::FormIntegerInput(name, required, value));
}

template<typename Callback>
void add_form_number(Form<Callback> & form, const std::string & name, bool required)
{
  form.addElement(mc_rtc::gui::FormNumberInput(name, required));
}

template<typename Callback>
void add_form_number(Form<Callback> & form, const std::string & name, bool required, double value)
{
  form.addElement(mc_rtc::gui::FormNumberInput(name, required, value));
}

template<typename Callback>
void add_form_string(Form<Callback> & form, const std::string & name, bool required)
{
  form.addElement(mc_rtc::gui::FormStringInput(name, required));
}

template<typename Callback>
void add_form_string(Form<Callback> & form, const std::string & name, bool required, const std::string & value)
{
  form.addElement(mc_rtc::gui::FormStringInput(name, required, value));
}

template<typename Callback>
void add_form_number_array(Form<Callback> & form, const std::string & name, bool required, bool fixed_size)
{
  form.addElement(mc_rtc::gui::FormArrayInput<std::vector<double>>(name, required, fixed_size));
}

template<typename Callback>
void add_form_number_array(Form<Callback> & form,
                           const std::string & name,
                           bool required,
                           const std::vector<double> & value,
                           bool fixed_size)
{
  form.addElement(mc_rtc::gui::FormArrayInput(name, required, value, fixed_size));
}

template<typename Callback>
void add_form_string_array(Form<Callback> & form, const std::string & name, bool required, bool fixed_size)
{
  form.addElement(mc_rtc::gui::FormArrayInput<std::vector<std::string>>(name, required, fixed_size));
}

template<typename Callback>
void add_form_string_array(Form<Callback> & form,
                           const std::string & name,
                           bool required,
                           const std::vector<std::string> & value,
                           bool fixed_size)
{
  form.addElement(mc_rtc::gui::FormArrayInput(name, required, value, fixed_size));
}
