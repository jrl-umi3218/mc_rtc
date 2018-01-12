#pragma once

#include <mc_rtc/GUIState.h>

namespace mc_rtc
{

namespace gui
{

template<typename T>
Input<T>::Input(std::vector<std::string> labels)
: labels_(std::move(labels))
{
}

template<typename T>
Input<T>::Input(std::vector<std::string> labels, T min, T max)
: labels_(std::move(labels)),
  has_min_max_(true), min_(min), max_(max)
{
}

template<typename T>
void Input<T>::addData(mc_rtc::Configuration out) const
{
  // Exclude labels when size == 1
  if(labels_.size() > 1)
  {
    out.add("labels", labels_);
  }
  if(has_min_max_)
  {
    out.add("min", min_);
    out.add("max", max_);
  }
}

template<typename T>
ComboList<T>::ComboList(std::vector<T> values)
: values_(std::move(values))
{
}

template<typename T>
void ComboList<T>::addData(mc_rtc::Configuration out) const
{
  out.add("values", values_);
}

namespace
{
  template<size_t i, size_t rem, typename ... Args,
    typename std::enable_if<rem == 0, int>::type = 0>
  void expand_form(const std::vector<std::string> &,
                   mc_rtc::Configuration &, const std::tuple<Args...> &)
  {
  }

  template<size_t i, size_t rem, typename ... Args,
    typename std::enable_if<rem != 0, int>::type = 0>
  void expand_form(const std::vector<std::string> & labels,
                   mc_rtc::Configuration & out, const std::tuple<Args...> & els)
  {
    assert(i < labels.size());
    using T = typename std::tuple_element<i, std::tuple<Args...>>::type;
    auto elOut = out.add(labels[i]);
    elOut.add("type", T::type);
    std::get<i>(els).addData(elOut);
    expand_form<i+1,rem-1>(labels, out, els);
  }
}

template<typename ... Args>
Form::Form(const std::vector<std::string> & labels, Args ... elements_)
{
  assert(sizeof...(elements_) == labels.size());
  std::tuple<Args...> elements = std::make_tuple(elements_...);
  addData = [labels, elements](mc_rtc::Configuration out)
  {
    auto els = out.add("elements");
    expand_form<0,std::tuple_size<decltype(elements)>::value>(labels, els, elements);
  };
}

template<typename T>
Element<T>::Element(const std::vector<std::string> & names)
: Element(names, nullptr, nullptr)
{
}

template<typename T>
Element<T>::Element(const std::vector<std::string> & names,
                    get_fn_t get_fn)
: Element(names, &get_fn, nullptr)
{
}

template<typename T>
Element<T>::Element(const std::vector<std::string> & names,
                    set_fn_t set_fn)
: Element(names, nullptr, &set_fn)
{
}

template<typename T>
Element<T>::Element(const std::vector<std::string> & names,
                    get_fn_t get_fn, set_fn_t set_fn)
: Element(names, &get_fn, &set_fn)
{
}

template<typename T>
Element<T>::Element(const std::vector<std::string> & names,
                    get_fn_t * get_fn, set_fn_t * set_fn)
{
  if(!names.size())
  {
    throw std::runtime_error("Cannot add an element without names");
  }
  if(names.size() > 1)
  {
    categories_.resize(names.size() - 1);
    for(size_t i = 0; i < names.size() - 1; ++i)
    {
      categories_[i] = names[i];
    }
  }
  name_ = names.back();
  if(get_fn) { has_get_fn_ = true; get_fn_ = *get_fn; }
  if(set_fn) { has_set_fn_ = true; set_fn_ = *set_fn; }
}

template<typename T>
void Element<T>::sendData(mc_rtc::Configuration out) const
{
  if(has_get_fn_)
  {
    out.add("data", get_fn_());
  }
}

template<typename T>
bool Element<T>::handleRequest(const mc_rtc::Configuration & in) const
{
  if(has_set_fn_)
  {
    try
    {
      set_fn_(in);
    }
    catch(const mc_rtc::Configuration::Exception exc)
    {
      LOG_ERROR("Method invokation failed to handle data" << std::endl
                << in.dump(true))
      LOG_ERROR(exc.what())
      return false;
    }
    return true;
  }
  return false;
}

template<typename T, typename InteractionT>
void StateBuilder::addElement(const T & element,
                              const InteractionT & interaction)
{
  static_assert(std::is_base_of<ElementBase, T>::value, "You can only add elements that derive from mc_rtc::gui::Element");
  static_assert(std::is_base_of<InteractionType, InteractionT>::value, "You can only use interactions that derive from mc_rtc::gui::InteractionType");
  static_assert(InteractionT::template is_compatible<typename T::data_t>(), "This interaction type is not compatible with this element type");

  // FIXME We store element/interaction two times which is slightly
  // inefficient, what we want is to store those in a type-agnostic manner
  // and retrieve them in the other callbacks

  // Initialize category if empty
  auto & category = elements_[element.categories_];
  const auto & name = element.name_;
  category[name] = [element, interaction](mc_rtc::Configuration & out)
  {
    auto out_name = out.add(element.name_);
    if(element.has_get_fn_)
    {
      element.sendData(out_name);
    }
    out_name.add("GUI").add("type", InteractionT::type);
    interaction.addData(out_name("GUI"));
    if(element.has_set_fn_)
    {
      out_name.add("SET");
    }
  };

  // Set the methods part of the element
  if(element.has_set_fn_)
  {
    auto & category = methods_[element.categories_];
    const auto & name = element.name_;
    category[name] = [element](const mc_rtc::Configuration & in)
    {
      if(!in.has("data"))
      {
        LOG_ERROR("Attempted to invoke a method without data in the input")
        return false;
      }
      return element.handleRequest(in("data"));
    };
  }
}

} // namespace gui

} // namepsace mc_rtc
