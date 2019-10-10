/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/GUIState.h>

namespace mc_rtc
{

namespace gui
{

template<typename T>
void StateBuilder::addElement(const std::vector<std::string> & category, T element)
{
  addElement(category, ElementsStacking::Vertical, element);
}

template<typename T>
void StateBuilder::addElement(const std::vector<std::string> & category, ElementsStacking stacking, T element)
{
  addElementImpl(category, stacking, element);
}

template<typename T>
void StateBuilder::addElementImpl(const std::vector<std::string> & category,
                                  ElementsStacking stacking,
                                  T element,
                                  size_t rem)
{
  static_assert(std::is_base_of<Element, T>::value, "You can only add elements that derive from the Element class");
  Category & cat = getCategory(category);
  auto it = std::find_if(cat.elements.begin(), cat.elements.end(),
                         [&element](const ElementStore & el) { return el().name() == element.name(); });
  if(it != cat.elements.end())
  {
    LOG_ERROR("An element named " << element.name() << " already exists in " << cat2str(category))
    LOG_WARNING("Discarding request to add this element")
    return;
  }
  cat.elements.emplace_back(element, cat, stacking);
  if(rem == 0)
  {
    cat.id += 1;
  }
}

template<typename T, typename... Args>
void StateBuilder::addElement(const std::vector<std::string> & category, T element, Args... args)
{
  addElement(category, ElementsStacking::Vertical, element, args...);
}

template<typename T, typename... Args>
void StateBuilder::addElement(const std::vector<std::string> & category,
                              ElementsStacking stacking,
                              T element,
                              Args... args)
{
  size_t rem = stacking == ElementsStacking::Vertical ? 0 : sizeof...(args);
  addElementImpl(category, stacking, element, rem);
  addElement(category, stacking, args...);
}

template<typename T>
StateBuilder::ElementStore::ElementStore(T self, const Category & category, ElementsStacking stacking)
{
  self.id(category.id);
  // FIXME In C++14 we could have T && self and move it into the lambda
  element = [self]() mutable -> Element & { return self; };
  if(stacking == ElementsStacking::Vertical)
  {
    write = [](Element & el, mc_rtc::MessagePackBuilder & builder) {
      builder.start_array(T::write_size());
      builder.write(el.name());
      builder.write(static_cast<typename std::underlying_type<Elements>::type>(T::type));
      builder.write(); // No stack id
      static_cast<T &>(el).write(builder);
      builder.finish_array();
    };
  }
  else
  {
    write = [](Element & el, mc_rtc::MessagePackBuilder & builder) {
      builder.start_array(T::write_size());
      builder.write(el.name());
      builder.write(static_cast<typename std::underlying_type<Elements>::type>(T::type));
      builder.write(el.id()); // No stack id
      static_cast<T &>(el).write(builder);
      builder.finish_array();
    };
  }
  handleRequest = [](Element & el, const mc_rtc::Configuration & data) {
    T & el_ = static_cast<T &>(el);
    return el_.handleRequest(data);
  };
}

} // namespace gui

} // namespace mc_rtc
