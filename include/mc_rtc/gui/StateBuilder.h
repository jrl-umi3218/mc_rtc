/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui/elements.h>

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
  static constexpr int8_t PROTOCOL_VERSION = 1;

  /** Constructor */
  StateBuilder();

  /** Add a given element
   *
   * T must derive from Element
   *
   * \param category Category of the element
   *
   * \param element Element added to the GUI
   */
  template<typename T>
  void addElement(const std::vector<std::string> & category, T element);

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

  /** Remove all elements */
  void reset();

  /** Remove a given category */
  void removeCategory(const std::vector<std::string> & category);

  /** Remove a single element */
  void removeElement(const std::vector<std::string> & category, const std::string & name);

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

private:
  template<typename T>
  void addElementImpl(const std::vector<std::string> & category, ElementsStacking stacking, T element, size_t rem = 0);

  /** Holds static data for the GUI */
  mc_rtc::Configuration data_;
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

    template<typename T>
    ElementStore(T self, const Category & category, ElementsStacking stacking);
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
  };
  Category elements_;

  /** Get a category
   *
   * Returns false and parent category if the category does
   * not exist, true and the request category otherwise
   *
   * \p category Requested category
   *
   * \p getParent If true returns the parent category,
   * otherwise returns the category
   */
  std::pair<bool, Category &> getCategory(const std::vector<std::string> & category, bool getParent);

  /** Get a category, creates it if does not exist */
  Category & getCategory(const std::vector<std::string> & category);

  /** Update the GUI data state for a given category */
  void update(mc_rtc::MessagePackBuilder & builder, Category & category);

  std::string cat2str(const std::vector<std::string> & category);
};

} // namespace gui

} // namespace mc_rtc

#include "StateBuilder.hpp"
