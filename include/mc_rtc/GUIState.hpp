#pragma once

#include <mc_rtc/GUIState.h>

namespace mc_rtc
{

namespace gui
{

template<typename T>
Input<T>::Input(T min, T max)
: has_min_max_(true), min_(min), max_(max)
{
}

template<typename T>
void Input<T>::addData(mc_rtc::Configuration & out) const
{
  if(has_min_max_)
  {
    out.add("min", min_);
    out.add("max", max_);
  }
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
void Element<T>::sendData(mc_rtc::Configuration & out) const
{
  if(has_get_fn_)
  {
    out.add(name_, get_fn_());
  }
}

template<typename T>
bool Element<T>::handleRequest(const mc_rtc::Configuration & in) const
{
  if(has_set_fn_)
  {
    set_fn_(in);
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

  // Set the state part of the element
  if(element.has_get_fn_)
  {
    // Initialize category if empty
    auto & category = elements_[element.categories_];
    const auto & name = element.name_;
    category[name] = [element, interaction](mc_rtc::Configuration & out)
    {
      element.sendData(out);
      auto inter = out.add(element.name_ + "_MC_RTC_GUI");
      inter.add("type", InteractionT::type);
      interaction.addData(inter);
    };
  }

  // Set the methods part of the element
  if(element.has_set_fn_)
  {
    state_.methods_changed = true;
    {
      auto & category = methods_[element.categories_];
      const auto & name = element.name_;
      category[name] = [element](const mc_rtc::Configuration & in)
      {
        if(in.has("data"))
        {
          LOG_ERROR("Attempted to invoke a method without data in the input")
          return false;
        }
        return element.handleRequest(in("data"));
      };
    }
    {
      auto & category = update_methods_[element.categories_];
      const auto & name = element.name_;
      category[name] = [interaction](mc_rtc::Configuration & out)
      {
        out.add("type", InteractionT::type);
        interaction.addData(out);
      };
    }
  }
}

} // namespace gui

} // namepsace mc_rtc
