#pragma once

#include <mc_rtc/gui_api.h>

#include <mc_rbdyn/configuration_io.h>

namespace mc_rtc
{

namespace gui
{

  struct MC_RTC_GUI_DLLAPI Color
  {
    Color() {}
    Color(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {}
    Color(float r, float g, float b) : r(r), g(g), b(b) {}
    float r = 0.0;
    float g = 0.0;
    float b = 0.0;
    float a = 1.0;
  };

  struct MC_RTC_GUI_DLLAPI Element
  {
    Element(const std::string & name);

    const std::string & name() const { return name_; }

    void addData(mc_rtc::Configuration &) {}
    void addGUI(mc_rtc::Configuration &) {}
    bool handleRequest(const mc_rtc::Configuration &) { return false; }
  protected:
    std::string name_;
  };

  enum class Elements
  {
    Label = 0,
    ArrayLabel,
    Button,
    Toggle,
    StringInput,
    IntegerInput,
    NumberInput,
    ArrayInput,
    ComboInput,
    Point3D,
    Rotation,
    Transform,
    Wrench,
    Schema,
    Form
  };

  template<typename GetT>
  struct MC_RTC_GUI_DLLAPI Label : public Element
  {
    static constexpr auto type = Elements::Label;

    Label(const std::string & name, GetT get_fn);

    void addData(mc_rtc::Configuration & data);
  private:
    GetT get_fn_;
  };

  template<typename GetT>
  Label<GetT> makeLabel(const std::string & name, GetT get_fn)
  {
    return Label<GetT>(name, get_fn);
  }

  template<typename GetT>
  struct MC_RTC_GUI_DLLAPI ArrayLabel : public Label<GetT>
  {
    static constexpr auto type = Elements::ArrayLabel;

    using Label<GetT>::Label;

    ArrayLabel(const std::string & name, const std::vector<std::string> & labels, GetT get_fn);

    void addGUI(mc_rtc::Configuration & gui);
  private:
    std::vector<std::string> labels_;
  };

  template<typename GetT>
  ArrayLabel<GetT> makeArrayLabel(const std::string & name, GetT get_fn)
  {
    return ArrayLabel<GetT>(name, get_fn);
  }

  /** Used to build a GUI state from multiple objects */
  struct MC_RTC_GUI_DLLAPI StateBuilder
  {
    /** Constructor */
    StateBuilder();

    /** Add a given element
     *
     * T must derive from Element
     *
     * \param category Category of the element
     * \param element Element added to the GUI
     */
    template<typename T>
    void addElement(const std::vector<std::string> & category, T element);

    /** Remove all elements */
    void reset();

    /** Remove a given category */
    void removeCategory(const std::vector<std::string> & category);

    /** Remove a single element */
    void removeElement(const std::vector<std::string> & category, const std::string & name);

    /** Update the GUI state */
    const mc_rtc::Configuration & update();

    /** Handle a request */
    bool handleRequest(const std::vector<std::string> & category,
                       const std::string & name,
                       const mc_rtc::Configuration & data);

    /** Access static data store */
    mc_rtc::Configuration data();
  private:
    mc_rtc::Configuration state_;
    struct ElementStore
    {
      const Element & operator()() const;
      Element & operator()();
      std::function<Element&()> element;
      void (*addData)(Element &, mc_rtc::Configuration &);
      void (*addGUI)(Element &, mc_rtc::Configuration &);
      bool (*handleRequest)(Element &, const mc_rtc::Configuration &);

      template<typename T>
      ElementStore(T self);
    };
    struct Category
    {
      std::vector<ElementStore> elements;
      std::map<std::string, Category> sub;
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
    std::pair<bool, Category&> getCategory(const std::vector<std::string> & category, bool getParent);

    /** Get a category, creates it if does not exist */
    Category & getCategory(const std::vector<std::string> & category);

    void update(Category & category, mc_rtc::Configuration out);

    std::string cat2str(const std::vector<std::string> & category);
  };

} // namespace gui

} // namespace mc_rtc

#include "GUIState.hpp"
