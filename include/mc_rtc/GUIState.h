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

  struct StateBuilder;

  /** InteractionType base type
   *
   * By specifying an InteractionType associated to an Element one can
   * indicate to the GUI how the element should be displayed. This may
   * imply a contract on the content of the element.
   *
   * For example, a Point3D must contain an Eigen::Vector3d, a Wrench must
   * contain an sva::ForceVecd but an Input or a Button have no such
   * limitations.
   *
   */
  struct InteractionType
  {
    /** Check if that interaction type is compatible with a given input
     * type */
    template<typename T>
    inline static constexpr bool is_compatible() { return true; }

    /** Add information to the output */
    inline void addData(mc_rtc::Configuration &) const {}
  };

  /** InteractionType with restricted compatibility */
  template<typename T>
  struct StrictInteractionType : public InteractionType
  {
    template<typename U,
      typename std::enable_if<!std::is_same<U, T>::value, int>::type = 0>
    inline static constexpr bool is_compatible() { return false; }

    template<typename U,
      typename std::enable_if<std::is_same<U, T>::value, int>::type = 0>
    inline static constexpr bool is_compatible() { return true; }
  };

  /** A label */
  struct MC_RTC_GUI_DLLAPI Label : public InteractionType
  {
    static constexpr auto type = "Label";

    Label(bool is_vector = false);

    bool is_numeric;
    bool is_integer;
    bool is_vector;

    void addData(mc_rtc::Configuration & out) const;
  };

  /** A button */
  struct Button : public InteractionType
  {
    static constexpr auto type = "Button";
  };

  struct Toggle : public InteractionType 
  {
    static constexpr auto type = "Toggle";
  };

  template<typename T>
  struct Input : public InteractionType
  {
    static constexpr auto type = "Input";
    /** Default constructor, no limit on the input */
    Input() = default;

    /** Input with limits on the value
     *
     * \param min Minimum value
     *
     * \param max Maximum value
     *
     */
    Input(T min, T max);

    void addData(mc_rtc::Configuration & out) const;
  private:
    bool has_min_max_ = false;
    T min_;
    T max_;
  };

  struct Point3D : public StrictInteractionType<Eigen::Vector3d>
  {
    static constexpr auto type = "Point3D";
  };

  struct Transform : public StrictInteractionType<sva::PTransformd>
  {
    static constexpr auto type = "Transform";
  };

  struct Wrench : public StrictInteractionType<sva::ForceVecd>
  {
    static constexpr auto type = "Wrench";
  };

  /** Element tag type */
  struct ElementBase {};

  /** Generic structure holding all information for the state builder */
  template<typename T>
  struct Element : public ElementBase
  {
    friend struct StateBuilder;
    /** The data type of the element */
    using data_t = typename std::decay<T>::type;
    /** The type of a function that can retrieve this element */
    using get_fn_t = std::function<data_t()>;
    /** The type of a function that can set this element */
    using set_fn_t = std::function<void(const data_t&)>;

    /** Simple constructor without setter or getter
     *
     * \param names A list of names representing the category/sub-category/name
     * of the element, this is used to uniquely identify the element. For
     * example, given a CoMTask one could pass: {"Tasks", "CoM {{robot name}}",
     * "stiffness"}. It is up to the GUI implementation to use this information
     * as it sees fit.
     *
     */
    Element(const std::vector<std::string> & names);

    /** Constructor for a read-only element
     *
     * \param names See the simple constructor
     *
     * \param get_fn Read function
     *
     */
    Element(const std::vector<std::string> & names,
            get_fn_t get_fn);

    /** Constructor for a write-only element
     *
     * \param names See the simple constructor
     *
     * \param set_fn Write function
     *
     */
    Element(const std::vector<std::string> & names,
            set_fn_t set_fn);

    /** Constructor for a read-write element
     *
     * \param names See the simple constructor
     *
     * \param get_fn Read function
     *
     * \param set_fn Write function
     *
     */
    Element(const std::vector<std::string> & names,
            get_fn_t get_fn, set_fn_t set_fn);
  protected:
    /** Internal constructor, used to simplify the writing of other constructors */
    Element(const std::vector<std::string> & names,
            get_fn_t * get_fn, set_fn_t * set_fn);

    std::vector<std::string> categories_ = {};
    std::string name_ = "";
    get_fn_t get_fn_ = [](){ return data_t{}; };
    set_fn_t set_fn_ = [](const data_t &){};

    /** Put data out */
    void sendData(mc_rtc::Configuration & out) const;

    /** Handle request */
    bool handleRequest(const mc_rtc::Configuration & in) const;
  private:
    bool has_get_fn_ = false;
    bool has_set_fn_ = false;
  };

  /** This structure holds information regarding the current status of the
   * GUI */
  struct MC_RTC_GUI_DLLAPI State
  {
    mc_rtc::Configuration state;

    bool methods_changed = false;
    mc_rtc::Configuration methods;

    /** Returns the provided state category
     *
     * If the category does not exist yet, create it
     *
     * \param category Category to retrieve
     *
     */
    mc_rtc::Configuration getStateCategory(const std::vector<std::string> & category);

    /** Returns the provided method category
     *
     * \see getStateCategory
     *
     */
    mc_rtc::Configuration getMethodCategory(const std::vector<std::string> & category);
  private:
    mc_rtc::Configuration getCategory(mc_rtc::Configuration & config, const std::vector<std::string> & category);
  };

  /** This structure is used to build a GUI state from the current status
   * of a variety of objects */
  struct MC_RTC_GUI_DLLAPI StateBuilder
  {
    template<typename T, typename InteractionT = Label>
    void addElement(const T & element,
                    const InteractionT & interaction = Label{});

    /** Remove all elements */
    void reset();

    /** Remove all elements nested under a given category
     *
     *  This has no effect if the category does not exist
     *
     * \param category Category of elements to remove
     *
     */
    void removeCategory(const std::vector<std::string> & category);

    /** Remove a single element using its unique name
     *
     *  This has no effect if the element does not exist
     *
     * \param names Names of the element (see Element documentation)
     *
     */
    void removeElement(const std::vector<std::string> & names);

    /** Rebuild the state */
    const State & updateState();

    /** Call a method
     *
     * Returns false if the call failed
     *
     */
    bool callMethod(const mc_rtc::Configuration & method);
  private:
    using element_cb_t = std::function<void(mc_rtc::Configuration&)>;
    using element_t = std::map<std::string, element_cb_t>;
    using category_t = std::vector<std::string>;
    /** Store calls to update the state */
    std::map<category_t, element_t> elements_;

    using method_cb_t = std::function<bool(const mc_rtc::Configuration&)>;
    using element_method_t = std::map<std::string, method_cb_t>;
    /** Store calls to process method calls */
    std::map<category_t, element_method_t> methods_;

    /** Store calls to update the methods' state */
    std::map<category_t, element_t> update_methods_;

    State state_;
  };

} // namespace gui

} // namespace mc_rtc

#include "GUIState.hpp"
