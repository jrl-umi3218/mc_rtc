/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/logging.h>

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
    log::error("An element named {} already exists in {}", element.name(), cat2str(category));
    log::warning("Discarding request to add this element");
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

template<typename T, typename... Args>
void StateBuilder::addXYPlot(const std::string & name, T data, Args... args)
{
  addXYPlot(name, {}, {}, {}, data, args...);
}

template<typename T, typename... Args>
void StateBuilder::addXYPlot(const std::string & name, plot::AxisConfiguration xConfig, T data, Args... args)
{
  addXYPlot(name, xConfig, {}, {}, data, args...);
}

template<typename T, typename... Args>
void StateBuilder::addXYPlot(const std::string & name,
                             plot::AxisConfiguration xConfig,
                             plot::AxisConfiguration yLeftConfig,
                             T data,
                             Args... args)
{
  addXYPlot(name, xConfig, yLeftConfig, {}, data, args...);
}

template<typename T, typename... Args>
void StateBuilder::addXYPlot(const std::string & name,
                             plot::AxisConfiguration xConfig,
                             plot::AxisConfiguration yLeftConfig,
                             plot::AxisConfiguration yRightConfig,
                             T data,
                             Args... args)
{
  static_assert(plot::is_2d<T, Args...>(), "All provided plots in an XY plot must provide 2d data");
  if(plots_.count(name) != 0)
  {
    log::error("A plot titled {} is still active", name);
    log::warning("Discarding request to add this plot");
    return;
  }
  // One entry for the type, the plot id, the name, the x and y axis configs, the data and one entry per plot
  uint64_t sz = 7 + sizeof...(args);
  uint64_t id = ++plot_id_;
  plot_callback_t cb = [data, id, sz, xConfig, yLeftConfig, yRightConfig](mc_rtc::MessagePackBuilder & builder,
                                                                          const std::string & name) {
    builder.start_array(sz);
    builder.write(static_cast<uint64_t>(plot::Plot::XY));
    builder.write(id);
    builder.write(name);
    xConfig.write(builder);
    yLeftConfig.write(builder);
    yRightConfig.write(builder);
    data.write(builder);
  };
  plots_[name] = makePlotCallback(cb, args...);
}

template<typename T, typename... Args>
void StateBuilder::addPlot(const std::string & name, T abscissa, Args... args)
{
  addPlot(name, abscissa, {}, {}, args...);
}

template<typename T, typename... Args>
void StateBuilder::addPlot(const std::string & name, T abscissa, plot::AxisConfiguration yLeftConfig, Args... args)
{
  addPlot(name, abscissa, yLeftConfig, {}, args...);
}

template<typename T, typename... Args>
void StateBuilder::addPlot(const std::string & name,
                           T abscissa,
                           plot::AxisConfiguration yLeftConfig,
                           plot::AxisConfiguration yRightConfig,
                           Args... args)
{
  static_assert(plot::is_Abscissa<T>(), "You must provide an Abscissa instance first");
  static_assert(plot::is_not_Abscissa<Args...>(), "Only one Abscissa can be provided to addPlot");
  if(plots_.count(name) != 0)
  {
    log::error("A plot titled {} is still active", name);
    log::warning("Discarding request to add this plot");
    return;
  }
  // One entry for the type, the plot id, the name, the abscissa and both axis configs plus one entry per plot
  uint64_t sz = 6 + sizeof...(args);
  uint64_t id = ++plot_id_;
  plot_callback_t cb = [abscissa, id, sz, yLeftConfig, yRightConfig](mc_rtc::MessagePackBuilder & builder,
                                                                     const std::string & name) {
    builder.start_array(sz);
    builder.write(static_cast<uint64_t>(plot::Plot::Standard));
    builder.write(id);
    builder.write(name);
    abscissa.write(builder);
    yLeftConfig.write(builder);
    yRightConfig.write(builder);
  };
  plots_[name] = makePlotCallback(cb, args...);
}

template<typename T>
StateBuilder::plot_callback_t StateBuilder::makePlotCallback(plot_callback_t callback, T plot)
{
  return [callback, plot](mc_rtc::MessagePackBuilder & builder, const std::string & name) {
    callback(builder, name);
    plot.write(builder);
    builder.finish_array();
  };
}

template<typename T, typename... Args>
StateBuilder::plot_callback_t StateBuilder::makePlotCallback(plot_callback_t callback, T plot, Args... args)
{
  plot_callback_t cb = [callback, plot](mc_rtc::MessagePackBuilder & builder, const std::string & name) {
    callback(builder, name);
    plot.write(builder);
  };
  return makePlotCallback(cb, args...);
}

} // namespace gui

} // namespace mc_rtc
