/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/configuration_io.h>
#include <mc_rtc/GUITypes.h>
#include <mc_rtc/gui_api.h>

namespace mc_rtc
{

namespace gui
{

struct MC_RTC_GUI_DLLAPI Element
{
  const std::string & name() const
  {
    return name_;
  }

  /** Access the stack id of an element
   *
   * Elements that should be displayed on the same line share the same id
   */
  int id() const
  {
    return id_;
  }

  /** Set the stack id of an element */
  void id(int idIn)
  {
    id_ = idIn;
  }

  void addData(mc_rtc::Configuration &) {}
  void addGUI(mc_rtc::Configuration &) {}
  bool handleRequest(const mc_rtc::Configuration &)
  {
    return false;
  }

  /** Invalid element, used for Python bindings */
  Element() {}

protected:
  Element(const std::string & name);

  std::string name_;

  int id_;
};

enum class Elements
{
  Label = 0,
  ArrayLabel,
  Button,
  Checkbox,
  StringInput,
  IntegerInput,
  NumberInput,
  NumberSlider,
  ArrayInput,
  ComboInput,
  DataComboInput,
  Point3D,
  Trajectory,
  Rotation,
  Transform,
  Schema,
  Form,
  Polygon,
  Force,
  Arrow,
  XYTheta
};

enum class ElementsStacking
{
  Vertical = 0,
  Horizontal
};

template<typename GetT>
struct DataElement : public Element
{
  void addData(mc_rtc::Configuration & data);

  DataElement(const std::string & name, GetT get_fn);

  /** Invalid element */
  DataElement() {}

protected:
  GetT get_fn_;
};

template<typename ElementT, typename Callback>
struct CallbackElement : public ElementT
{
  bool handleRequest(const mc_rtc::Configuration &);

  template<typename... Args>
  CallbackElement(const std::string & name, Callback cb, Args &&... args);

  /** Invalid element */
  CallbackElement() {}

protected:
  Callback cb_;
};

template<typename ElementT, typename Callback>
struct VoidCallbackElement : public CallbackElement<ElementT, Callback>
{
  bool handleRequest(const mc_rtc::Configuration &);

  template<typename... Args>
  VoidCallbackElement(const std::string & name, Callback cb, Args &&... args)
  : CallbackElement<ElementT, Callback>(name, cb, std::forward<Args>(args)...)
  {
  }

  /** Invalid element */
  VoidCallbackElement() {}
};

template<typename GetT>
struct LabelImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Label;

  LabelImpl(const std::string & name, GetT get_fn);

  /** Invalid element */
  LabelImpl() {}
};

template<typename GetT>
LabelImpl<GetT> Label(const std::string & name, GetT get_fn)
{
  return LabelImpl<GetT>(name, get_fn);
}

template<typename GetT>
struct ArrayLabelImpl : public LabelImpl<GetT>
{
  static constexpr auto type = Elements::ArrayLabel;

  ArrayLabelImpl(const std::string & name, GetT get_fn) : LabelImpl<GetT>(name, get_fn) {}

  ArrayLabelImpl(const std::string & name, const std::vector<std::string> & labels, GetT get_fn);

  /** Invalid element */
  ArrayLabelImpl() {}

  void addGUI(mc_rtc::Configuration & gui);

private:
  std::vector<std::string> labels_;
};

template<typename GetT>
ArrayLabelImpl<GetT> ArrayLabel(const std::string & name, GetT get_fn)
{
  return ArrayLabelImpl<GetT>(name, get_fn);
}

template<typename GetT>
ArrayLabelImpl<GetT> ArrayLabel(const std::string & name, const std::vector<std::string> & labels, GetT get_fn)
{
  return ArrayLabelImpl<GetT>(name, labels, get_fn);
}

template<typename Callback>
struct ButtonImpl : public VoidCallbackElement<Element, Callback>
{
  static constexpr auto type = Elements::Button;

  template<typename... Args>
  ButtonImpl(const std::string & name, Callback cb, Args &&... args)
  : VoidCallbackElement<Element, Callback>(name, cb, std::forward<Args>(args)...)
  {
  }

  /** Invalid element */
  ButtonImpl() {}
};

template<typename Callback>
ButtonImpl<Callback> Button(const std::string & name, Callback cb)
{
  return ButtonImpl<Callback>(name, cb);
}

template<typename GetT, typename Callback>
struct CheckboxImpl : public VoidCallbackElement<DataElement<GetT>, Callback>
{
  static constexpr auto type = Elements::Checkbox;

  CheckboxImpl(const std::string & name, GetT get_fn, Callback cb);

  /** Invalid element */
  CheckboxImpl() {}
};

template<typename GetT, typename Callback>
CheckboxImpl<GetT, Callback> Checkbox(const std::string & name, GetT get_fn, Callback cb)
{
  return CheckboxImpl<GetT, Callback>(name, get_fn, cb);
}

template<typename GetT, typename SetT>
struct CommonInputImpl : public CallbackElement<DataElement<GetT>, SetT>
{
  CommonInputImpl(const std::string & name, GetT get_fn, SetT set_fn);

  /** Invalid element */
  CommonInputImpl() {}
};

#define WRITE_INPUT_IMPL(NAME)                                                                                         \
  template<typename GetT, typename SetT>                                                                               \
  struct NAME##Impl : public CommonInputImpl<GetT, SetT>                                                               \
  {                                                                                                                    \
    static constexpr auto type = Elements::NAME;                                                                       \
    NAME##Impl(const std::string & name, GetT get_fn, SetT set_fn) : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn) \
    {                                                                                                                  \
    }                                                                                                                  \
    /** Invalid element */                                                                                             \
    NAME##Impl() {}                                                                                                    \
  };                                                                                                                   \
  template<typename GetT, typename SetT>                                                                               \
  NAME##Impl<GetT, SetT> NAME(const std::string & name, GetT get_fn, SetT set_fn)                                      \
  {                                                                                                                    \
    return NAME##Impl<GetT, SetT>(name, get_fn, set_fn);                                                               \
  }

WRITE_INPUT_IMPL(StringInput)
WRITE_INPUT_IMPL(IntegerInput)
WRITE_INPUT_IMPL(NumberInput)

#undef WRITE_INPUT_IMPL

template<typename GetT, typename SetT>
struct NumberSliderImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::NumberSlider;

  NumberSliderImpl(const std::string & name, GetT get_fn, SetT set_fn, double min, double max)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn), min_(min), max_(max)
  {
  }

  NumberSliderImpl() {}

  /** Add min/max to GUI information */
  void addGUI(mc_rtc::Configuration &);

private:
  double min_;
  double max_;
};

template<typename GetT, typename SetT>
NumberSliderImpl<GetT, SetT> NumberSlider(const std::string & name, GetT get_fn, SetT set_fn, double min, double max)
{
  return NumberSliderImpl<GetT, SetT>(name, get_fn, set_fn, min, max);
}

template<typename GetT, typename SetT>
struct ArrayInputImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::ArrayInput;
  ArrayInputImpl(const std::string & name, GetT get_fn, SetT set_fn)
  : CommonInputImpl<GetT, SetT>::CommonInputImpl(name, get_fn, set_fn)
  {
  }

  /** Array input with labels per-dimension */
  ArrayInputImpl(const std::string & name, const std::vector<std::string> & labels, GetT get_fn, SetT set_fn);

  /** Add labels to GUI information */
  void addGUI(mc_rtc::Configuration &);

  /** Invalid element */
  ArrayInputImpl() {}

private:
  std::vector<std::string> labels_;
};

template<typename GetT, typename SetT>
ArrayInputImpl<GetT, SetT> ArrayInput(const std::string & name, GetT get_fn, SetT set_fn)
{
  return ArrayInputImpl<GetT, SetT>(name, get_fn, set_fn);
}

template<typename GetT, typename SetT>
ArrayInputImpl<GetT, SetT> ArrayInput(const std::string & name,
                                      const std::vector<std::string> & labels,
                                      GetT get_fn,
                                      SetT set_fn)
{
  return ArrayInputImpl<GetT, SetT>(name, labels, get_fn, set_fn);
}

template<typename GetT, typename SetT>
struct ComboInputImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::ComboInput;

  ComboInputImpl(const std::string & name, const std::vector<std::string> & values, GetT get_fn, SetT set_fn);

  /** Add valid values to GUI information */
  void addGUI(mc_rtc::Configuration & gui);

  /** Invalid element */
  ComboInputImpl() {}

private:
  std::vector<std::string> values_;
};

template<typename GetT, typename SetT>
ComboInputImpl<GetT, SetT> ComboInput(const std::string & name,
                                      const std::vector<std::string> & values,
                                      GetT get_fn,
                                      SetT set_fn)
{
  return ComboInputImpl<GetT, SetT>(name, values, get_fn, set_fn);
}

template<typename GetT, typename SetT>
struct DataComboInputImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::DataComboInput;

  DataComboInputImpl(const std::string & name, const std::vector<std::string> & data_ref, GetT get_fn, SetT set_fn);

  /** Add valid values to GUI information */
  void addGUI(mc_rtc::Configuration & gui);

  /** Invalid element */
  DataComboInputImpl() {}

private:
  std::vector<std::string> data_ref_;
};

template<typename GetT, typename SetT>
DataComboInputImpl<GetT, SetT> DataComboInput(const std::string & name,
                                              const std::vector<std::string> & values,
                                              GetT get_fn,
                                              SetT set_fn)
{
  return DataComboInputImpl<GetT, SetT>(name, values, get_fn, set_fn);
}

template<typename GetT>
struct Point3DROImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Point3D;

  Point3DROImpl(const std::string & name, const PointConfig & config, GetT get_fn);

  /** Add distinguishing elements to GUI information */
  void addGUI(mc_rtc::Configuration & gui);

  /** Invalid element */
  Point3DROImpl() {}

private:
  PointConfig config_;
};

template<typename GetT, typename SetT>
struct Point3DImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::Point3D;

  Point3DImpl(const std::string & name, const PointConfig & config, GetT get_fn, SetT set_fn);

  /** Invalid element */
  Point3DImpl() {}

  /** Add distinguishing elements to GUI information */
  void addGUI(mc_rtc::Configuration & gui);

private:
  PointConfig config_;
};

template<typename GetT>
Point3DROImpl<GetT> Point3D(const std::string & name, GetT get_fn)
{
  return Point3DROImpl<GetT>(name, PointConfig{}, get_fn);
}

template<typename GetT, typename SetT>
Point3DImpl<GetT, SetT> Point3D(const std::string & name, GetT get_fn, SetT set_fn)
{
  return Point3DImpl<GetT, SetT>(name, PointConfig{}, get_fn, set_fn);
}

template<typename GetT>
Point3DROImpl<GetT> Point3D(const std::string & name, const PointConfig & config, GetT get_fn)
{
  return Point3DROImpl<GetT>(name, config, get_fn);
}

template<typename GetT, typename SetT>
Point3DImpl<GetT, SetT> Point3D(const std::string & name, const PointConfig & config, GetT get_fn, SetT set_fn)
{
  return Point3DImpl<GetT, SetT>(name, config, get_fn, set_fn);
}

template<typename GetT>
struct TrajectoryImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Trajectory;

  TrajectoryImpl(const std::string & name, GetT get_fn);

  /** Invalid element */
  TrajectoryImpl() {}
};

template<typename GetT>
struct TrajectoryWithStyleImpl : public TrajectoryImpl<GetT>
{
  TrajectoryWithStyleImpl(const std::string & name, const LineConfig & config, GetT get_fn);

  /** Invalid element */
  TrajectoryWithStyleImpl() {}

  void addGUI(mc_rtc::Configuration & gui);

private:
  LineConfig config_;
};

template<typename GetT>
TrajectoryImpl<GetT> Trajectory(const std::string & name, GetT get_fn)
{
  return TrajectoryImpl<GetT>(name, get_fn);
}

template<typename GetT>
TrajectoryWithStyleImpl<GetT> Trajectory(const std::string & name, const LineConfig & config, GetT get_fn)
{
  return TrajectoryWithStyleImpl<GetT>(name, config, get_fn);
}

template<typename GetT>
struct PolygonImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Polygon;

  PolygonImpl(const std::string & name, const Color & color, GetT get_fn);

  /** Invalid element */
  PolygonImpl() {}

  void addGUI(mc_rtc::Configuration & gui);

private:
  Color color_;
};

template<typename GetT>
PolygonImpl<GetT> Polygon(const std::string & name, GetT get_fn)
{
  return PolygonImpl<GetT>(name, Color{0., 1., 0.}, get_fn);
}

template<typename GetT>
PolygonImpl<GetT> Polygon(const std::string & name, const Color & color, GetT get_fn)
{
  return PolygonImpl<GetT>(name, color, get_fn);
}

template<typename GetForce, typename GetSurface>
struct ForceImpl : public Element
{
  static constexpr auto type = Elements::Force;

  ForceImpl(const std::string & name, const ForceConfig & config, GetForce get_force_fn, GetSurface get_surface_fn);
  /** Invalid element */
  ForceImpl() {}

  void addData(mc_rtc::Configuration & data);

  void addGUI(mc_rtc::Configuration & gui);

private:
  ForceConfig config_;
  GetForce get_force_fn_;
  GetSurface get_surface_fn_;
};

template<typename GetForce, typename GetSurface>
ForceImpl<GetForce, GetSurface> Force(const std::string & name, GetForce get_force_fn, GetSurface get_surface_fn)
{
  return ForceImpl<GetForce, GetSurface>(name, ForceConfig{}, get_force_fn, get_surface_fn);
}

template<typename GetForce, typename GetSurface>
ForceImpl<GetForce, GetSurface> Force(const std::string & name,
                                      const ForceConfig & config,
                                      GetForce get_force_fn,
                                      GetSurface get_surface_fn)
{
  return ForceImpl<GetForce, GetSurface>(name, config, get_force_fn, get_surface_fn);
}

template<typename GetStart, typename GetEnd>
struct ArrowImpl : public Element
{
  static constexpr auto type = Elements::Arrow;

  ArrowImpl(const std::string & name, const ArrowConfig & config, GetStart get_start_fn, GetEnd get_end_fn);
  /** Invalid element */
  ArrowImpl(){};

  void addData(mc_rtc::Configuration & data);

  void addGUI(mc_rtc::Configuration & gui);

private:
  ArrowConfig config_;
  GetStart get_start_fn_;
  GetEnd get_end_fn_;
};

template<typename GetStart, typename GetEnd>
ArrowImpl<GetStart, GetEnd> Arrow(const std::string & name, GetStart get_start_fn, GetEnd get_end_fn)
{
  return ArrowImpl<GetStart, GetEnd>(name, ArrowConfig{}, get_start_fn, get_end_fn);
}

template<typename GetStart, typename GetEnd>
ArrowImpl<GetStart, GetEnd> Arrow(const std::string & name,
                                  const ArrowConfig & config,
                                  GetStart get_start_fn,
                                  GetEnd get_end_fn)
{
  return ArrowImpl<GetStart, GetEnd>(name, config, get_start_fn, get_end_fn);
}

template<typename GetT>
struct RotationROImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Rotation;

  RotationROImpl(const std::string & name, GetT get_fn) : DataElement<GetT>(name, get_fn) {}

  /** Add distinguishing elements to GUI information */
  void addGUI(mc_rtc::Configuration & gui);

  /** Invalid element */
  RotationROImpl() {}
};

template<typename GetT, typename SetT>
struct RotationImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::Rotation;

  RotationImpl(const std::string & name, GetT get_fn, SetT set_fn) : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn)
  {
  }

  /** Invalid element */
  RotationImpl() {}
};

template<typename GetT>
RotationROImpl<GetT> Rotation(const std::string & name, GetT get_fn)
{
  return RotationROImpl<GetT>(name, get_fn);
}

template<typename GetT, typename SetT>
RotationImpl<GetT, SetT> Rotation(const std::string & name, GetT get_fn, SetT set_fn)
{
  return RotationImpl<GetT, SetT>(name, get_fn, set_fn);
}

template<typename GetT>
struct TransformROImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Transform;

  TransformROImpl(const std::string & name, GetT get_fn) : DataElement<GetT>(name, get_fn) {}

  /** Add distinguishing elements to GUI information */
  void addGUI(mc_rtc::Configuration & gui);

  /** Invalid element */
  TransformROImpl() {}
};

template<typename GetT, typename SetT>
struct TransformImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::Transform;

  TransformImpl(const std::string & name, GetT get_fn, SetT set_fn) : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn)
  {
  }

  /** Invalid element */
  TransformImpl() {}
};

template<typename GetT>
TransformROImpl<GetT> Transform(const std::string & name, GetT get_fn)
{
  return TransformROImpl<GetT>(name, get_fn);
}

template<typename GetT, typename SetT>
TransformImpl<GetT, SetT> Transform(const std::string & name, GetT get_fn, SetT set_fn)
{
  return TransformImpl<GetT, SetT>(name, get_fn, set_fn);
}

template<typename GetT>
struct XYThetaROImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::XYTheta;

  XYThetaROImpl(const std::string & name, GetT get_fn) : DataElement<GetT>(name, get_fn) {}

  /** Add distinguishing elements to GUI information */
  void addGUI(mc_rtc::Configuration & gui);

  /** Invalid element */
  XYThetaROImpl() {}
};

template<typename GetT, typename SetT>
struct XYThetaImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::XYTheta;

  XYThetaImpl(const std::string & name, GetT get_fn, SetT set_fn) : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn) {}

  /** Invalid element */
  XYThetaImpl() {}
};

template<typename GetT>
XYThetaROImpl<GetT> XYTheta(const std::string & name, GetT get_fn)
{
  return XYThetaROImpl<GetT>(name, get_fn);
}

template<typename GetT, typename SetT>
XYThetaImpl<GetT, SetT> XYTheta(const std::string & name, GetT get_fn, SetT set_fn)
{
  return XYThetaImpl<GetT, SetT>(name, get_fn, set_fn);
}

template<typename Callback>
struct SchemaImpl : public CallbackElement<Element, Callback>
{
  static constexpr auto type = Elements::Schema;

  SchemaImpl(const std::string & name, const std::string & schema, Callback cb);

  void addGUI(mc_rtc::Configuration & gui);

private:
  std::string schema_;
};

template<typename Callback>
SchemaImpl<Callback> Schema(const std::string & name, const std::string & schema, Callback cb)
{
  return SchemaImpl<Callback>(name, schema, cb);
}

template<typename Callback>
struct FormImpl : public CallbackElement<Element, Callback>
{
  static constexpr auto type = Elements::Form;

  template<typename... Args>
  FormImpl(const std::string & name, Callback cb, Args &&... args);

  void addGUI(mc_rtc::Configuration & gui);

  template<typename T>
  void addElement(T && arg);

  /** Invalid element */
  FormImpl() {}

private:
  mc_rtc::Configuration gui_;
};

template<typename Derived>
struct FormInput
{
  void addGUI(mc_rtc::Configuration & gui);

  void addGUI_(mc_rtc::Configuration &) {}

  /** Invalid element */
  FormInput() {}

protected:
  FormInput(const std::string & name, bool required);

  std::string name_;
  bool required_;
};

template<typename T, Elements element>
struct FormDataInput : public FormInput<FormDataInput<T, element>>
{
  static constexpr auto type = element;

  FormDataInput(const std::string & name, bool required, const T & def)
  : FormInput<FormDataInput<T, element>>(name, required), def_(def)
  {
  }

  FormDataInput(const std::string & name, bool required) : FormInput<FormDataInput<T, element>>(name, required) {}

  void addGUI_(mc_rtc::Configuration & out)
  {
    out.add("default", def_);
  }

  /** Invalid element */
  FormDataInput() {}

private:
  T def_;
};

using FormCheckbox = FormDataInput<bool, Elements::Checkbox>;
using FormIntegerInput = FormDataInput<int, Elements::IntegerInput>;
using FormNumberInput = FormDataInput<double, Elements::NumberInput>;
using FormStringInput = FormDataInput<std::string, Elements::StringInput>;

template<typename T>
struct FormArrayInput : public FormInput<FormArrayInput<T>>
{
  static constexpr auto type = Elements::ArrayInput;

  FormArrayInput(const std::string & name, bool required, const T & def, bool fixed_size = true)
  : FormInput<FormArrayInput>(name, required), def_(def), fixed_size_(fixed_size)
  {
  }

  FormArrayInput(const std::string & name, bool required, bool fixed_size = false)
  : FormInput<FormArrayInput>(name, required), fixed_size_(fixed_size)
  {
  }

  void addGUI_(mc_rtc::Configuration & out)
  {
    out.add("fixed_size", fixed_size_);
    out.add("default", def_);
  }

  /** Invalid element */
  FormArrayInput() {}

private:
  T def_;
  bool fixed_size_;
};

struct MC_RTC_GUI_DLLAPI FormComboInput : public FormInput<FormComboInput>
{
  static constexpr auto type = Elements::ComboInput;

  FormComboInput(const std::string & name,
                 bool required,
                 const std::vector<std::string> & values,
                 bool send_index = false);

  void addGUI_(mc_rtc::Configuration & gui);

  /** Invalid element */
  FormComboInput() {}

private:
  std::vector<std::string> values_;
  bool send_index_ = false;
};

struct MC_RTC_GUI_DLLAPI FormDataComboInput : public FormInput<FormDataComboInput>
{
  static constexpr auto type = Elements::DataComboInput;

  FormDataComboInput(const std::string & name,
                     bool required,
                     const std::vector<std::string> & ref,
                     bool send_index = false);

  void addGUI_(mc_rtc::Configuration & gui);

  /** Invalid element */
  FormDataComboInput() {}

private:
  std::vector<std::string> ref_;
  bool send_index_;
};

template<typename Callback, typename... Args>
FormImpl<Callback> Form(const std::string & name, Callback cb, Args &&... args)
{
  return FormImpl<Callback>(name, cb, std::forward<Args>(args)...);
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

  template<typename T, typename... Args>
  void addElement(const std::vector<std::string> & category, T element, Args... args);

  template<typename T>
  void addElement(const std::vector<std::string> & category, ElementsStacking stacking, T element);

  template<typename T, typename... Args>
  void addElement(const std::vector<std::string> & category, ElementsStacking stacking, T element, Args... args);

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
  template<typename T>
  void addElementImpl(const std::vector<std::string> & category, ElementsStacking stacking, T element, size_t rem = 0);

  mc_rtc::Configuration state_;
  struct Category;
  struct MC_RTC_GUI_DLLAPI ElementStore
  {
    const Element & operator()() const;
    Element & operator()();
    std::function<Element &()> element;
    void (*addData)(Element &, mc_rtc::Configuration &);
    void (*addGUI)(Element &, mc_rtc::Configuration &);
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
  void update(Category & category, mc_rtc::Configuration out);

  /** Update the GUI elements information for a given element */
  void updateGUI(const std::vector<std::string> & category, ElementStore & el);

  std::string cat2str(const std::vector<std::string> & category);
};

} // namespace gui

} // namespace mc_rtc

#include "GUIState.hpp"
