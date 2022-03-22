/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/gui/plot.h>

#include <unordered_map>

namespace mc_rtc
{

namespace gui
{

/** Describe elements stacking policy for the client */
enum class ElementsStacking
{
  Vertical = 0,
  Horizontal
};

/** Used to build a GUI state from multiple objects */
struct MC_RTC_GUI_DLLAPI StateBuilder
{
  /** Increment this version when an update makes old client code incompatible
   *
   * Things that will break the client:
   * - Removing fields from an existing element
   * - Changing data requirements
   *
   * Things that should not affect the client:
   * - Adding fields to an existing Element
   * - Adding an Element type
   */
  static constexpr int8_t PROTOCOL_VERSION = 3;

  /** Constructor */
  StateBuilder();

  /** Add a given element
   *
   * \tparam T Must derive from Element
   *
   * \param category Category of the element
   *
   * \param element Element added to the GUI
   */
  template<typename T>
  void addElement(const std::vector<std::string> & category, T element);

  /** Add a given element
   *
   * \tparam SourceT Type of the source pointer
   *
   * \tparam T Must derive from Element
   *
   * \param source Source attached to this object
   *
   * \param category Category of the element
   *
   * \param element Element added to the GUI
   */
  template<typename SourceT, typename T>
  void addElement(SourceT * source, const std::vector<std::string> & category, T element);

  /** Add multiple elements to the same category at once
   *
   * \param category Category of the elements
   *
   * \param element Element added to the GUI
   *
   * \param args Other elements added to the GUI
   */
  template<typename T, typename... Args>
  void addElement(const std::vector<std::string> & category, T element, Args... args);

  /** Add multiple elements to the same category at once
   *
   * \param source Source attached to this object
   *
   * \param category Category of the elements
   *
   * \param element Element added to the GUI
   *
   * \param args Other elements added to the GUI
   */
  template<typename SourceT, typename T, typename... Args>
  void addElement(SourceT * source, const std::vector<std::string> & category, T element, Args... args);

  /** Add a given element and specify stacking
   *
   * T must derive from Element
   *
   * \param category Category of the element
   *
   * \param stacking Stacking direction
   *
   * \param element Element added to the GUI
   */
  template<typename T>
  void addElement(const std::vector<std::string> & category, ElementsStacking stacking, T element);

  /** Add a given element and specify stacking
   *
   * T must derive from Element
   *
   * \param source Source attached to this object
   *
   * \param category Category of the element
   *
   * \param stacking Stacking direction
   *
   * \param element Element added to the GUI
   */
  template<typename SourceT, typename T>
  void addElement(SourceT * source, const std::vector<std::string> & category, ElementsStacking stacking, T element);

  /** Add multiple elements to the same category at once with a specific stacking
   *
   * \param category Category of the elements
   *
   * \param element Element added to the GUI
   *
   * \param stacking Stacking direction
   *
   * \param args Other elements added to the GUI
   */
  template<typename T, typename... Args>
  void addElement(const std::vector<std::string> & category, ElementsStacking stacking, T element, Args... args);

  /** Add multiple elements to the same category at once with a specific stacking
   *
   * \param source Source attached to this object
   *
   * \param category Category of the elements
   *
   * \param element Element added to the GUI
   *
   * \param stacking Stacking direction
   *
   * \param args Other elements added to the GUI
   */
  template<typename SourceT, typename T, typename... Args>
  void addElement(SourceT * source,
                  const std::vector<std::string> & category,
                  ElementsStacking stacking,
                  T element,
                  Args... args);

  /** Checks if an element is already in the GUI
   *
   * \param category Category of the element
   * \param name Name of the element
   *
   * \return True if the element is already in the gui
   */
  bool hasElement(const std::vector<std::string> & category, const std::string & name);

  /** Remove all elements */
  void reset();

  /** Remove a given category */
  void removeCategory(const std::vector<std::string> & category);

  /** Remove a single element */
  void removeElement(const std::vector<std::string> & category, const std::string & name);

  /** Remove all elements attached to the given source
   *
   * One should prefer to remove a category or target a specific category, otherwise the whole GUI has to be searched to
   * find matching elements
   *
   * For example, if your source has added a category and a few elements in the root:
   * gui()->removeCategory({"MyCategory"});
   * gui()->removeElements({}, this);
   * Should be used instead of:
   * gui()->removeElements(this);
   */
  void removeElements(void * source);

  /** Remove all elements attached to the given source in the specified category
   *
   * \param category Category where elements will be searched
   *
   * \param source Source that will be searched
   *
   * \param recurse Also search for elements in sub-categories of the given category
   */
  void removeElements(const std::vector<std::string> & category, void * source, bool recurse = false);

  /** Add a plot identified by the provided name
   *
   * In this form, Args are expected to provide 2D data
   *
   * \param name Name of the plot
   *
   * \param xConfig Configuration for the left axis
   *
   * \param yLeftConfig Configuration for the left axis
   *
   * \param yRightConfig Configuration for the right axis
   *
   */
  template<typename... Args>
  void addXYPlot(const std::string & name,
                 plot::AxisConfiguration xConfig,
                 plot::AxisConfiguration yLeftConfig,
                 plot::AxisConfiguration yRightConfig,
                 Args... args);

  /** Add a plot identified by the provided name
   *
   * In this form, Args are expected to provide 2D data
   *
   * \param name Name of the plot
   *
   * \param xConfig Configuration for the left axis
   *
   * \param yLeftConfig Configuration for the left axis
   *
   */
  template<typename... Args>
  void addXYPlot(const std::string & name,
                 plot::AxisConfiguration xConfig,
                 plot::AxisConfiguration yLeftConfig,
                 Args... args);

  /** Add a plot identified by the provided name
   *
   * In this form, Args are expected to provide 2D data
   *
   * \param name Name of the plot
   *
   * \param xConfig Configuration for the left axis
   *
   * \param yLeftConfig Configuration for the left axis
   *
   */
  template<typename... Args>
  void addXYPlot(const std::string & name, plot::AxisConfiguration xConfig, Args... args);

  /** Add a plot identified by the provided name
   *
   * In this form, Args are expected to provide 2D data
   *
   * \param name Name of the plot
   *
   * \param xConfig Configuration for the left axis
   *
   * \param yLeftConfig Configuration for the left axis
   *
   */
  template<typename... Args>
  void addXYPlot(const std::string & name, Args... args);

  /** Add a plot identified by the provided name
   *
   * In this form, T is expected to provide an abscissa, the other parameters
   * are expected to provide Y-axis or 2D data
   *
   * \param name Name of the plot, this is a unique identifier
   *
   * \param abscissa Describe the abscissa axis
   *
   * \param yLeftConfig Configuration for the left axis
   *
   * \param yRightConfig Configuration for the right axis
   */
  template<typename T, typename... Args>
  void addPlot(const std::string & name,
               T abscissa,
               plot::AxisConfiguration yLeftConfig,
               plot::AxisConfiguration yRightConfig,
               Args... args);

  /** Add a plot identified by the provided name
   *
   * In this form, T is expected to provide an abscissa, the other parameters
   * are expected to provide Y-axis or 2D data
   *
   * \param name Name of the plot, this is a unique identifier
   *
   * \param abscissa Describe the abscissa axis
   *
   * \param yLeftConfig Configuration for the left axis
   */
  template<typename T, typename... Args>
  void addPlot(const std::string & name, T abscissa, plot::AxisConfiguration yLeftConfig, Args... args);

  /** Add a plot identified by the provided name
   *
   * In this form, T is expected to provide an abscissa, the other parameters
   * are expected to provide Y-axis or XY-axis data
   *
   * \param name Name of the plot, this is a unique identifier
   *
   * \param abscissa Describe the abscissa axis
   */
  template<typename T, typename... Args>
  void addPlot(const std::string & name, T abscissa, Args... args);

  /** Add more data to an existing plot
   *
   * T must be a Y-axis or 2D data
   *
   * No-op if the plot has not been added yet
   *
   * No-op if T a Y-axis and the plot was added with \ref addXYPlot
   *
   * \param name Name of the plot, it must have been added via \ref addPlot or \ref addXYPlot before
   *
   * \param data Extra data to be added
   *
   * \return True if the data was added, false otherwise
   *
   */
  template<typename T>
  bool addPlotData(const std::string & name, T data);

  /** Remove a plot identified by the provided name */
  void removePlot(const std::string & name);

  /** Update the GUI message
   *
   * \param data Will hold binary data representing the GUI
   *
   * \returns Effective size of the GUI message
   *
   */
  size_t update(std::vector<char> & data);

  /** Handle a request */
  bool handleRequest(const std::vector<std::string> & category,
                     const std::string & name,
                     const mc_rtc::Configuration & data);

  /** Access static data store
   *
   * This assumes you are accessing the data to modify and will trigger a
   * regeneration of the data portion of the message. For better performances
   * use sparsely.
   */
  mc_rtc::Configuration data();

  /** Return the number of elements in the GUI */
  inline size_t size() const
  {
    return elements_.size();
  }

private:
  template<typename T>
  void addElementImpl(void * source,
                      const std::vector<std::string> & category,
                      ElementsStacking stacking,
                      T element,
                      size_t rem = 0);

  /** Holds static data for the GUI */
  mc_rtc::Configuration data_;
  /** Callback used to write plot data into the GUI message */
  using plot_callback_function_t = std::function<void(mc_rtc::MessagePackBuilder &, const std::string &)>;
  struct PlotCallback
  {
    plot::Plot type;
    size_t msg_size;
    plot_callback_function_t callback;
  };
  /** A unique plot id used to identify plots with the same name
   *
   * This is mainly useful to restart a plot with the same name in a single iteration
   */
  uint64_t plot_id_ = 0;
  /** Holds all currently active plots */
  std::unordered_map<std::string, PlotCallback> plots_;
  /** True if data binary form needs to be generated again */
  bool update_data_ = true;
  /** Holds data's binary form */
  std::vector<char> data_buffer_;
  /** Holds data's binary size */
  size_t data_buffer_size_ = 0;
  struct Category;
  struct MC_RTC_GUI_DLLAPI ElementStore
  {
    const Element & operator()() const;
    Element & operator()();
    std::function<Element &()> element;
    void (*write)(Element &, mc_rtc::MessagePackBuilder &);
    bool (*handleRequest)(Element &, const mc_rtc::Configuration &);
    void * source;

    template<typename T>
    ElementStore(T self, const Category & category, ElementsStacking stacking, void * source);
  };
  struct Category
  {
    std::string name;
    std::vector<ElementStore> elements;
    std::vector<Category> sub;
    /** If the category has a sub-category of the requested name returns an
     * iterator to it, otherwise returns end() */
    std::vector<Category>::iterator find(const std::string & name);
    /** For each category, keeps track of the line id for next elements added */
    int id;
    /** Returns the number of elements in this category and its sub-categories */
    inline size_t size() const
    {
      size_t s = 0;
      for(const auto & c : sub)
      {
        s += c.size();
      }
      return s + elements.size();
    }
  };
  Category elements_;

  /** Get a category
   *
   * Returns nullptr if the category does not exist
   *
   * \p category Requested category
   *
   * \p depth Only consider the first \p depth elements in \p category (up-to category's size)
   */
  Category * getCategory(const std::vector<std::string> & category, size_t depth = std::numeric_limits<size_t>::max());

  /** Get a category, creates it if does not exist */
  Category & getOrCreateCategory(const std::vector<std::string> & category);

  /** Update the GUI data state for a given category */
  void update(mc_rtc::MessagePackBuilder & builder, Category & category);

  /** Remove all elements associated to the given in the given category */
  void removeElements(Category & category, void * source);

  std::string cat2str(const std::vector<std::string> & category);

  void addPlotData(PlotCallback &) {}

  template<typename T, typename... Args>
  void addPlotData(PlotCallback & callback, T plot, Args... args);
};

} // namespace gui

} // namespace mc_rtc

#include "StateBuilder.hpp"
