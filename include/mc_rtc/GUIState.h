/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/configuration_io.h>
#include <mc_rtc/GUIDetails.h>
#include <mc_rtc/GUITypes.h>
#include <mc_rtc/gui_api.h>

namespace mc_rtc
{

namespace gui
{

struct MC_RTC_GUI_DLLAPI Element
{
  /** Name of the element */
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

  /** Returns the size of the array representing the widget in the MessagePack
   *
   * By default, it includes the widget name and type and SID.
   */
  static constexpr size_t write_size()
  {
    return 3;
  }

  /** Write the widget to a MessagePackBuilder
   *
   * Writes nothing, the default data is written by GUIState since it knows if
   * the sid is relevant.
   */
  void write(mc_rtc::MessagePackBuilder &) {}

  /** Take care of answering request from the client */
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

/** An enumeration of available elements */
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

/** Describe elements stacking policy for the client */
enum class ElementsStacking
{
  Vertical = 0,
  Horizontal
};

/** A generic element to write data to the GUI
 *
 * This is a common building block for other elements
 */
template<typename GetT>
struct DataElement : public Element
{
  static constexpr size_t write_size()
  {
    return Element::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(get_fn_());
  }

  /** Constructor */
  DataElement(const std::string & name, GetT get_fn) : Element(name), get_fn_(get_fn) {}

  /** Invalid element */
  DataElement() {}

protected:
  GetT get_fn_;
};

/** A generic element to handle request from the client
 *
 * This is a common building block for other elements.
 *
 */
template<typename ElementT, typename Callback>
struct CallbackElement : public ElementT
{
  bool handleRequest(const mc_rtc::Configuration & data)
  {
    cb_(data);
    return true;
  }

  template<typename... Args>
  CallbackElement(const std::string & name, Callback cb, Args &&... args)
  : ElementT(name, std::forward<Args>(args)...), cb_(cb)
  {
  }

  /** Invalid element */
  CallbackElement() {}

protected:
  Callback cb_;
};

/** A generic element to handle request from the client when no data is required
 *
 * This is a common building block for other elements
 *
 */
template<typename ElementT, typename Callback>
struct VoidCallbackElement : public CallbackElement<ElementT, Callback>
{
  bool handleRequest(const mc_rtc::Configuration &)
  {
    this->cb_();
    return true;
  }

  template<typename... Args>
  VoidCallbackElement(const std::string & name, Callback cb, Args &&... args)
  : CallbackElement<ElementT, Callback>(name, cb, std::forward<Args>(args)...)
  {
  }

  /** Invalid element */
  VoidCallbackElement() {}
};

/** Label should display data
 *
 * \tparam GetT Can return anything that can be serialized but the element will work best with numbers or strings
 *
 * \see ArrayLabelImpl to display arrays
 */
template<typename GetT>
struct LabelImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Label;

  LabelImpl(const std::string & name, GetT get_fn) : DataElement<GetT>(name, get_fn) {}

  /** Invalid element */
  LabelImpl() {}
};

/** Helper function to create a Label element */
template<typename GetT>
LabelImpl<GetT> Label(const std::string & name, GetT get_fn)
{
  return LabelImpl<GetT>(name, get_fn);
}

/** ArrayLabel should display data in an array form
 *
 * Labels can be provided optionally
 *
 * \tparam GetT should return anything that can be serialized as an array
 *
 * \see LabelImpl to display simple data
 */
template<typename GetT>
struct ArrayLabelImpl : public LabelImpl<GetT>
{
  static constexpr auto type = Elements::ArrayLabel;

  ArrayLabelImpl(const std::string & name, GetT get_fn) : LabelImpl<GetT>(name, get_fn) {}

  ArrayLabelImpl(const std::string & name, const std::vector<std::string> & labels, GetT get_fn)
  : LabelImpl<GetT>(name, get_fn), labels_(labels)
  {
  }

  /** Invalid element */
  ArrayLabelImpl() {}

  static constexpr size_t write_size()
  {
    return LabelImpl<GetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & writer)
  {
    LabelImpl<GetT>::write(writer);
    writer.write(labels_);
  }

private:
  std::vector<std::string> labels_;
};

/** Helper function to build an ArrayLabelImpl (no labels) */
template<typename GetT>
ArrayLabelImpl<GetT> ArrayLabel(const std::string & name, GetT get_fn)
{
  return ArrayLabelImpl<GetT>(name, get_fn);
}

/** Helper function to build an ArrayLabelImpl (with labels) */
template<typename GetT>
ArrayLabelImpl<GetT> ArrayLabel(const std::string & name, const std::vector<std::string> & labels, GetT get_fn)
{
  return ArrayLabelImpl<GetT>(name, labels, get_fn);
}

/** Button should provide a clickable button
 *
 * When the button is clicked, Callback() is called
 *
 * \tparam Callback The callback that will be called when clicking the button
 *
 */
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

/** Helper function to create a ButtonImpl */
template<typename Callback>
ButtonImpl<Callback> Button(const std::string & name, Callback cb)
{
  return ButtonImpl<Callback>(name, cb);
}

/** Checkbox display a toggable checkbox
 *
 * \tparam GetT Should return a boolean providing the state of the button
 *
 * \tparam Callback When the checkbox is called, this is called
 *
 */
template<typename GetT, typename Callback>
struct CheckboxImpl : public VoidCallbackElement<DataElement<GetT>, Callback>
{
  static constexpr auto type = Elements::Checkbox;

  CheckboxImpl(const std::string & name, GetT get_fn, Callback cb)
  : VoidCallbackElement<DataElement<GetT>, Callback>(name, cb, get_fn)
  {
    static_assert(details::CheckReturnType<GetT, bool>::value,
                  "Checkbox element getter callback must return a boolean");
  }

  /** Invalid element */
  CheckboxImpl() {}
};

/** Helper function to create a Checkbox */
template<typename GetT, typename Callback>
CheckboxImpl<GetT, Callback> Checkbox(const std::string & name, GetT get_fn, Callback cb)
{
  return CheckboxImpl<GetT, Callback>(name, get_fn, cb);
}

/** Helper class to save on typing */
template<typename GetT, typename SetT>
struct CommonInputImpl : public CallbackElement<DataElement<GetT>, SetT>
{
  CommonInputImpl(const std::string & name, GetT get_fn, SetT set_fn)
  : CallbackElement<DataElement<GetT>, SetT>(name, set_fn, get_fn)
  {
  }

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

/** NumberSlider should display a number slider with minimum
 * and maximum values specified
 *
 * \tparam GetT Returns a numeric type
 *
 * \tparam SetT Called when the slider is manipulated by the
 * client
 */
template<typename GetT, typename SetT>
struct NumberSliderImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::NumberSlider;

  NumberSliderImpl(const std::string & name, GetT get_fn, SetT set_fn, double min, double max)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn), min_(min), max_(max)
  {
  }

  NumberSliderImpl() {}

  static constexpr size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 2;
  }

  void write(mc_rtc::MessagePackBuilder & writer)
  {
    CommonInputImpl<GetT, SetT>::write(writer);
    writer.write(min_);
    writer.write(max_);
  }

private:
  double min_;
  double max_;
};

/** Helper function to create a NumberSliderImpl */
template<typename GetT, typename SetT>
NumberSliderImpl<GetT, SetT> NumberSlider(const std::string & name, GetT get_fn, SetT set_fn, double min, double max)
{
  return NumberSliderImpl<GetT, SetT>(name, get_fn, set_fn, min, max);
}

/** ArrayInput should display an array (\see ArrayLabel) and
 * allow editing
 *
 * \tparam GetT Returns an array-like object
 *
 * \tparam SetT Called when the client inputs a new value for
 * the array
 *
 */
template<typename GetT, typename SetT>
struct ArrayInputImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::ArrayInput;

  /** Constructor (no labels) */
  ArrayInputImpl(const std::string & name, GetT get_fn, SetT set_fn) : ArrayInputImpl(name, {}, get_fn, set_fn) {}

  /** Constructor with labels per-dimension */
  ArrayInputImpl(const std::string & name, const std::vector<std::string> & labels, GetT get_fn, SetT set_fn)
  : CommonInputImpl<GetT, SetT>::CommonInputImpl(name, get_fn, set_fn), labels_(labels)
  {
  }

  static constexpr size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & writer)
  {
    CommonInputImpl<GetT, SetT>::write(writer);
    writer.write(labels_);
  }

  /** Invalid element */
  ArrayInputImpl() {}

private:
  std::vector<std::string> labels_;
};

/** Helper function to create an ArrayInput element (no labels) */
template<typename GetT, typename SetT>
ArrayInputImpl<GetT, SetT> ArrayInput(const std::string & name, GetT get_fn, SetT set_fn)
{
  return ArrayInputImpl<GetT, SetT>(name, get_fn, set_fn);
}

/** Helper function to create an ArrayInput element (with labels) */
template<typename GetT, typename SetT>
ArrayInputImpl<GetT, SetT> ArrayInput(const std::string & name,
                                      const std::vector<std::string> & labels,
                                      GetT get_fn,
                                      SetT set_fn)
{
  return ArrayInputImpl<GetT, SetT>(name, labels, get_fn, set_fn);
}

/** ComboInput should display a combo box with the set of choices provided by values
 *
 * \tparam GetT Should return the current choice
 *
 * \tparam SetT Should accept the choice made by the user
 */
template<typename GetT, typename SetT>
struct ComboInputImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::ComboInput;

  ComboInputImpl(const std::string & name, const std::vector<std::string> & values, GetT get_fn, SetT set_fn)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn), values_(values)
  {
    static_assert(details::CheckReturnType<GetT, std::string>::value,
                  "ComboInput element getter callback should return an std::string");
  }

  static constexpr size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & writer)
  {
    CommonInputImpl<GetT, SetT>::write(writer);
    writer.write(values_);
  }

  /** Invalid element */
  ComboInputImpl() {}

private:
  std::vector<std::string> values_;
};

/** Helper function to create a ComboInputImpl */
template<typename GetT, typename SetT>
ComboInputImpl<GetT, SetT> ComboInput(const std::string & name,
                                      const std::vector<std::string> & values,
                                      GetT get_fn,
                                      SetT set_fn)
{
  return ComboInputImpl<GetT, SetT>(name, values, get_fn, set_fn);
}

/** DataComboInput should behave like ComboInput but the data source is stored
 * in the GUI data store
 *
 * \tparam GetT Should return the current choice
 *
 * \tparam SetT Should accept the choice made by the user
 */
template<typename GetT, typename SetT>
struct DataComboInputImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::DataComboInput;

  DataComboInputImpl(const std::string & name, const std::vector<std::string> & data_ref, GetT get_fn, SetT set_fn)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn), data_ref_(data_ref)
  {
  }

  static constexpr size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CommonInputImpl<GetT, SetT>::write(builder);
    builder.write(data_ref_);
  }

  /** Invalid element */
  DataComboInputImpl() {}

private:
  std::vector<std::string> data_ref_;
};

/** Helper function to build a DataComboInputImpl */
template<typename GetT, typename SetT>
DataComboInputImpl<GetT, SetT> DataComboInput(const std::string & name,
                                              const std::vector<std::string> & values,
                                              GetT get_fn,
                                              SetT set_fn)
{
  return DataComboInputImpl<GetT, SetT>(name, values, get_fn, set_fn);
}

/** Point3D should display a 3D point in the environment
 *
 * A PointConfig is provided to control how the point should be displayed
 *
 * With this variant, the point cannot be edited
 *
 * It will also trigger an ArrayLabel with {"x", "y", "z"} labels
 *
 * \tparam GetT Should return an Eigen::Vector3d
 */
template<typename GetT>
struct Point3DROImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Point3D;

  Point3DROImpl(const std::string & name, const PointConfig & config, GetT get_fn)
  : DataElement<GetT>(name, get_fn), config_(config)
  {
    static_assert(details::CheckReturnType<GetT, Eigen::Vector3d>::value,
                  "Point3D element position callback must return an Eigen::Vector3d");
  }

  static constexpr size_t write_size()
  {
    return DataElement<GetT>::write_size() + 1 + PointConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    DataElement<GetT>::write(builder);
    builder.write(true); // Read-only
    config_.write(builder);
  }

  /** Invalid element */
  Point3DROImpl() {}

private:
  PointConfig config_;
};

/** Point3D should display a 3D point in the environment
 *
 * A PointConfig is provided to control how the point is displayed
 *
 * With this variant, the point can be edited
 *
 * It will also trigger an ArrayInput with {"x", "y", "z"} labels
 *
 * \tparam GetT Should return an Eigen::Vector3d
 *
 * \tparam SetT Will be called when the point is moved or the ArrayInput is triggered
 */
template<typename GetT, typename SetT>
struct Point3DImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::Point3D;

  Point3DImpl(const std::string & name, const PointConfig & config, GetT get_fn, SetT set_fn)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn), config_(config)
  {
    static_assert(details::CheckReturnType<GetT, Eigen::Vector3d>::value,
                  "Point3D element position callback must return an Eigen::Vector3d");
  }

  /** Invalid element */
  Point3DImpl() {}

  static constexpr size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 1 + PointConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CommonInputImpl<GetT, SetT>::write(builder);
    builder.write(false); // Not read-only
    config_.write(builder);
  }

private:
  PointConfig config_;
};

/** Helper function to create a Point3DROImpl */
template<typename GetT>
Point3DROImpl<GetT> Point3D(const std::string & name, GetT get_fn)
{
  return Point3DROImpl<GetT>(name, {}, get_fn);
}

/** Helper function to create a Point3DImpl */
template<typename GetT, typename SetT>
Point3DImpl<GetT, SetT> Point3D(const std::string & name, GetT get_fn, SetT set_fn)
{
  return Point3DImpl<GetT, SetT>(name, {}, get_fn, set_fn);
}

/** Helper function to create a Point3DROImpl with configuration */
template<typename GetT>
Point3DROImpl<GetT> Point3D(const std::string & name, const PointConfig & config, GetT get_fn)
{
  return Point3DROImpl<GetT>(name, config, get_fn);
}

/** Helper function to create a Point3DImpl with configuration */
template<typename GetT, typename SetT>
Point3DImpl<GetT, SetT> Point3D(const std::string & name, const PointConfig & config, GetT get_fn, SetT set_fn)
{
  return Point3DImpl<GetT, SetT>(name, config, get_fn, set_fn);
}

/** Trajectory can represent a pre-defined trajectory or a real-time trajectory
 * depending on type of data returned by the callback
 *
 * Styling information is provided to inform the client about how to display the trajectory
 *
 * \tparam GetT Should return either an sva::PTransformd or an Eigen::Vector3d
 * (real-time trajectory) or a vector of these types (pre-defined trajectory)
 *
 */
template<typename GetT>
struct TrajectoryImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Trajectory;

  TrajectoryImpl(const std::string & name, const LineConfig & config, GetT get_fn)
  : DataElement<GetT>(name, get_fn), config_(config)
  {
    static_assert(details::CheckReturnType<GetT, Eigen::Vector3d, sva::PTransformd, std::vector<Eigen::Vector3d>,
                                           std::vector<sva::PTransformd>>::value,
                  "Trajectory element data callback must return either an Eigen::Vector3d, an sva::PTransformd or an "
                  "std::vector of either types");
  }

  /** Invalid element */
  TrajectoryImpl() {}

  constexpr static size_t write_size()
  {
    return DataElement<GetT>::write_size() + LineConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    DataElement<GetT>::write(builder);
    config_.write(builder);
  }

private:
  LineConfig config_;
};

/** Function helper to get a TrajectoryImpl */
template<typename GetT>
TrajectoryImpl<GetT> Trajectory(const std::string & name, GetT get_fn)
{
  return TrajectoryImpl<GetT>(name, {}, get_fn);
}

/** Function helper to get a TrajectoryImpl */
template<typename GetT>
TrajectoryImpl<GetT> Trajectory(const std::string & name, const LineConfig & config, GetT get_fn)
{
  return TrajectoryImpl<GetT>(name, config, get_fn);
}

/** Polygon should display a polygon or a set of polygons
 *
 * A color is provided to display the polygon(s)
 *
 * \tparam GetT Should return an std::vector<Eigen::Vector3d> (one polygon) or an
 * std::vector<std::vector<Eigen:Vector3d>> (list of polygons)
 *
 */
template<typename GetT>
struct PolygonImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Polygon;

  PolygonImpl(const std::string & name, const Color & color, GetT get_fn)
  : DataElement<GetT>(name, get_fn), color_(color)
  {
    static_assert(
        details::CheckReturnType<GetT, std::vector<Eigen::Vector3d>, std::vector<std::vector<Eigen::Vector3d>>>::value,
        "Polygon element data callback must return either an std::vector of Eigen::Vector3d or an std::vector of "
        "std::vector3d of Eigen::Vector3d");
  }

  /** Invalid element */
  PolygonImpl() {}

  static constexpr size_t write_size()
  {
    return DataElement<GetT>::write_size() + Color::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    DataElement<GetT>::write(builder);
    color_.write(builder);
  }

private:
  Color color_;
};

/** Helper function to build a PolygonImpl */
template<typename GetT>
PolygonImpl<GetT> Polygon(const std::string & name, GetT get_fn)
{
  return PolygonImpl<GetT>(name, {}, get_fn);
}

/** Helper function to build a PolygonImpl */
template<typename GetT>
PolygonImpl<GetT> Polygon(const std::string & name, const Color & color, GetT get_fn)
{
  return PolygonImpl<GetT>(name, color, get_fn);
}

/** Force should display a force vector in 3D environment
 *
 * The display style of the element can be configured \see ForceConfig
 *
 * Additionally an ArrayLabel with labels {"cx", "cy", "cz", "fx", "fy", "fz"} is created
 *
 * \tparam GetForce Should return an sva::ForceVecd
 *
 * \tparam GetSurface Should return an sva::PTransformd where the force will be displayed
 */
template<typename GetForce, typename GetSurface>
struct ForceImpl : public Element
{
  static constexpr auto type = Elements::Force;

  ForceImpl(const std::string & name, const ForceConfig & config, GetForce get_force_fn, GetSurface get_surface_fn)
  : Element(name), get_force_fn_(get_force_fn), get_surface_fn_(get_surface_fn), config_(config)
  {
    static_assert(details::CheckReturnType<GetForce, sva::ForceVecd>::value,
                  "Force element force callback must return an sva::ForceVecd");
    static_assert(details::CheckReturnType<GetSurface, sva::PTransformd>::value,
                  "Force element surface callback must return an sva::PTransformd");
  }

  static constexpr size_t write_size()
  {
    return Element::write_size() + 2 + ForceConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    Element::write(builder);
    builder.write(get_force_fn_());
    builder.write(get_surface_fn_());
    config_.write(builder);
  }

private:
  GetForce get_force_fn_;
  GetSurface get_surface_fn_;
  ForceConfig config_;
};

/** Helper function to get a ForceImpl */
template<typename GetForce, typename GetSurface>
ForceImpl<GetForce, GetSurface> Force(const std::string & name, GetForce get_force_fn, GetSurface get_surface_fn)
{
  return ForceImpl<GetForce, GetSurface>(name, {}, get_force_fn, get_surface_fn);
}

/** Helper function to get a ForceImpl */
template<typename GetForce, typename GetSurface>
ForceImpl<GetForce, GetSurface> Force(const std::string & name,
                                      const ForceConfig & config,
                                      GetForce get_force_fn,
                                      GetSurface get_surface_fn)
{
  return ForceImpl<GetForce, GetSurface>(name, config, get_force_fn, get_surface_fn);
}

/** Arrow should display an arrow from the point at the start to the point at the end
 *
 * An ArrowConfig can be provided to specify how the arrow should be displayed
 *
 * \tparam GetStart Returns an Eigen::Vector3d representing the starting point
 *
 * \tparam GetEnd Returns an Eigen::Vector3d representing the end point
 *
 */
template<typename GetStart, typename GetEnd>
struct ArrowImpl : public Element
{
  static constexpr auto type = Elements::Arrow;

  ArrowImpl(const std::string & name, const ArrowConfig & config, GetStart get_start_fn, GetEnd get_end_fn)
  : Element(name), get_start_fn_(get_start_fn), get_end_fn_(get_end_fn), config_(config)
  {
    static_assert(details::CheckReturnType<GetStart, Eigen::Vector3d>::value,
                  "Arrow element start callback must return an Eigen::Vector3d");
    static_assert(details::CheckReturnType<GetEnd, Eigen::Vector3d>::value,
                  "Arrow element end callback must return an Eigen::Vector3d");
  }

  /** Invalid element */
  ArrowImpl(){};

  constexpr static size_t write_size()
  {
    return Element::write_size() + 2 + ArrowConfig::write_size();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    Element::write(builder);
    builder.write(get_start_fn_());
    builder.write(get_end_fn_());
    config_.write(builder);
  }

private:
  GetStart get_start_fn_;
  GetEnd get_end_fn_;
  ArrowConfig config_;
};

/** Helper function to create an ArrowImpl */
template<typename GetStart, typename GetEnd>
ArrowImpl<GetStart, GetEnd> Arrow(const std::string & name, GetStart get_start_fn, GetEnd get_end_fn)
{
  return ArrowImpl<GetStart, GetEnd>(name, {}, get_start_fn, get_end_fn);
}

/** Helper function to create an ArrowImpl */
template<typename GetStart, typename GetEnd>
ArrowImpl<GetStart, GetEnd> Arrow(const std::string & name,
                                  const ArrowConfig & config,
                                  GetStart get_start_fn,
                                  GetEnd get_end_fn)
{
  return ArrowImpl<GetStart, GetEnd>(name, config, get_start_fn, get_end_fn);
}

/** Rotation display a widget that shows the rotation
 *
 * This rotation is not editable.
 *
 * This will also create a quaternion ArrayLabel with labels {"qw", "qx", "qy", "qz"}
 *
 * \tparam GetT Must return an sva::PTransformd (to display the rotation somewhere)
 */
template<typename GetT>
struct RotationROImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Rotation;

  RotationROImpl(const std::string & name, GetT get_fn) : DataElement<GetT>(name, get_fn)
  {
    static_assert(details::CheckReturnType<GetT, sva::PTransformd>::value,
                  "RotationROImpl getter should return an sva::PTransformd");
  }

  constexpr static size_t write_size()
  {
    return DataElement<GetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    DataElement<GetT>::write(builder);
    builder.write(true); // Is read-only
  }

  /** Invalid element */
  RotationROImpl() {}
};

/** Rotation display a widget that shows the rotation
 *
 * This rotation is editable.
 *
 * This will also create a quaternion ArrayInput with labels {"qw", "qx", "qy", "qz"}
 *
 * \tparam GetT Must return an sva::PTransformd (to display the rotation somewhere)
 *
 * \tparam SetT Should accept a rotation as an Eigen::Quaterniond
 */
template<typename GetT, typename SetT>
struct RotationImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::Rotation;

  RotationImpl(const std::string & name, GetT get_fn, SetT set_fn) : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn)
  {
    static_assert(details::CheckReturnType<GetT, sva::PTransformd>::value,
                  "RotationImpl getter should return an sva::PTransformd");
  }

  constexpr static size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CommonInputImpl<GetT, SetT>::write(builder);
    builder.write(false); // Is read-only
  }

  /** Invalid element */
  RotationImpl() {}
};

/** Helper function to create a Rotation element (read-only) */
template<typename GetT>
RotationROImpl<GetT> Rotation(const std::string & name, GetT get_fn)
{
  return RotationROImpl<GetT>(name, get_fn);
}

/** Helper function to create a Rotation element (writable) */
template<typename GetT, typename SetT>
RotationImpl<GetT, SetT> Rotation(const std::string & name, GetT get_fn, SetT set_fn)
{
  return RotationImpl<GetT, SetT>(name, get_fn, set_fn);
}

/** Transform display a widget that shows a 6D frame
 *
 * This transformation is not editable.
 *
 * This will also create an ArrayLabel with labels {"qw", "qx", "qy",
 * "qz", "tx", "ty", "tz"}
 *
 * \tparam GetT Must return an sva::PTransformd
 *
 */
template<typename GetT>
struct TransformROImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Transform;

  TransformROImpl(const std::string & name, GetT get_fn) : DataElement<GetT>(name, get_fn)
  {
    static_assert(details::CheckReturnType<GetT, sva::PTransformd>::value,
                  "TransformImpl getter should return an sva::PTransformd");
  }

  constexpr static size_t write_size()
  {
    return DataElement<GetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    DataElement<GetT>::write(builder);
    builder.write(true); // Is read-only
  }

  /** Invalid element */
  TransformROImpl() {}
};

/** Transform display a widget that shows a 6D frame
 *
 * This transformation is editable.
 *
 * This will also create an ArrayInput with labels {"qw", "qx", "qy",
 * "qz", "tx", "ty", "tz"}
 *
 * \tparam GetT Must return an sva::PTransformd
 *
 * \tparam SetT Should accept an sva::PTransformd
 *
 */
template<typename GetT, typename SetT>
struct TransformImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::Transform;

  TransformImpl(const std::string & name, GetT get_fn, SetT set_fn) : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn)
  {
    static_assert(details::CheckReturnType<GetT, sva::PTransformd>::value,
                  "TransformImpl getter should return an sva::PTransformd");
  }

  constexpr static size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CommonInputImpl<GetT, SetT>::write(builder);
    builder.write(false); // Is read-only
  }

  /** Invalid element */
  TransformImpl() {}
};

/** Helper function to create a Transform element (read-only) */
template<typename GetT>
TransformROImpl<GetT> Transform(const std::string & name, GetT get_fn)
{
  return TransformROImpl<GetT>(name, get_fn);
}

/** Helper function to create a Transform element (editable) */
template<typename GetT, typename SetT>
TransformImpl<GetT, SetT> Transform(const std::string & name, GetT get_fn, SetT set_fn)
{
  return TransformImpl<GetT, SetT>(name, get_fn, set_fn);
}

/** An XYTheta element represents an oriented point in the XY plane.
 *
 * Altitude can be provided optionally.
 *
 * This element is not editable
 *
 * \tparam GetT Should return a double array of size 3 (x, y, theta) or 4 (x, y, theta, z)
 *
 */
template<typename GetT>
struct XYThetaROImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::XYTheta;

  XYThetaROImpl(const std::string & name, GetT get_fn) : DataElement<GetT>(name, get_fn) {}

  /** Invalid element */
  XYThetaROImpl() {}

  constexpr static size_t write_size()
  {
    return DataElement<GetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    DataElement<GetT>::write(builder);
    builder.write(true); // Is read-only
  }
};

/** An XYTheta element represents an oriented point in the XY plane.
 *
 * Altitude can be provided optionally.
 *
 * This element is editable
 *
 * \tparam GetT Should return a double array of size 3 (x, y, theta) or 4 (x, y, theta, z)
 *
 */
template<typename GetT, typename SetT>
struct XYThetaImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::XYTheta;

  XYThetaImpl(const std::string & name, GetT get_fn, SetT set_fn) : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn) {}

  constexpr static size_t write_size()
  {
    return CommonInputImpl<GetT, SetT>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CommonInputImpl<GetT, SetT>::write(builder);
    builder.write(false); // Is read-only
  }

  /** Invalid element */
  XYThetaImpl() {}
};

/** Helper function to create an XYTheta element (read-only) */
template<typename GetT>
XYThetaROImpl<GetT> XYTheta(const std::string & name, GetT get_fn)
{
  return XYThetaROImpl<GetT>(name, get_fn);
}

/** Helper function to create an XYTheta element */
template<typename GetT, typename SetT>
XYThetaImpl<GetT, SetT> XYTheta(const std::string & name, GetT get_fn, SetT set_fn)
{
  return XYThetaImpl<GetT, SetT>(name, get_fn, set_fn);
}

/** Should display a form using the data provided in the configured schema folder
 *
 * The schema folder should be relative to mc_rtc install path
 *
 * \tparam Callback Called with form data when the user completes the form
 *
 */
template<typename Callback>
struct SchemaImpl : public CallbackElement<Element, Callback>
{
  static constexpr auto type = Elements::Schema;

  SchemaImpl(const std::string & name, const std::string & schema, Callback cb)
  : CallbackElement<Element, Callback>(name, cb), schema_(schema)
  {
  }

  static constexpr size_t write_size()
  {
    return CallbackElement<Element, Callback>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CallbackElement<Element, Callback>::write(builder);
    builder.write(schema_);
  }

private:
  std::string schema_;
};

/** Helper function to create a Schema element */
template<typename Callback>
SchemaImpl<Callback> Schema(const std::string & name, const std::string & schema, Callback cb)
{
  return SchemaImpl<Callback>(name, schema, cb);
}

/** Create a user-defined form
 *
 * A form is composed of 1 or more elements (FormInput) that can be required for the form completion (this must be
 * checked on client side)
 *
 * \tparam Callback Will be called when the form is completed on client side
 *
 */
template<typename Callback>
struct FormImpl : public CallbackElement<Element, Callback>
{
  static constexpr auto type = Elements::Form;

  template<typename... Args>
  FormImpl(const std::string & name, Callback cb, Args &&... args) : CallbackElement<Element, Callback>(name, cb)
  {
    mc_rtc::MessagePackBuilder builder(data_);
    count_ = sizeof...(Args);
    write_elements(builder, std::forward<Args>(args)...);
    data_size_ = builder.finish();
  }

  template<typename T>
  void addElement(T && element)
  {
    count_ += 1;
    std::vector<char> data = data_;
    mc_rtc::MessagePackBuilder builder(data_);
    builder.write_object(data.data(), data_size_);
    builder.start_array(element.write_size());
    element.write(builder);
    builder.finish_array();
    data_size_ = builder.finish();
  }

  static constexpr size_t write_size()
  {
    return CallbackElement<Element, Callback>::write_size() + 1;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    CallbackElement<Element, Callback>::write(builder);
    builder.start_array(count_);
    builder.write_object(data_.data(), data_size_);
    builder.finish_array();
  }

  /** Invalid element */
  FormImpl() {}

private:
  template<typename... Args>
  void write_elements(mc_rtc::MessagePackBuilder &, Args &&...)
  {
  }

  template<typename Arg, typename... Args>
  void write_elements(mc_rtc::MessagePackBuilder & builder, Arg && element, Args &&... args)
  {
    builder.start_array(element.write_size());
    element.write(builder);
    builder.finish_array();
    write_elements(builder, std::forward<Args>(args)...);
  }

  size_t count_;
  std::vector<char> data_;
  size_t data_size_;
};

template<typename Derived, Elements element>
struct FormElement
{
  static constexpr auto type = element;

  static constexpr size_t write_size()
  {
    return 3 + Derived::write_size_();
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(name_);
    builder.write(static_cast<typename std::underlying_type<Elements>::type>(type));
    builder.write(required_);
    static_cast<Derived &>(*this).write_(builder);
  }

  /** Invalid element */
  FormElement() {}

protected:
  FormElement(const std::string & name, bool required) : name_(name), required_(required) {}

  std::string name_;
  bool required_;
};

template<typename T, Elements element>
struct FormDataInput : public FormElement<FormDataInput<T, element>, element>
{
  FormDataInput(const std::string & name, bool required, const T & def)
  : FormElement<FormDataInput<T, element>, element>(name, required), def_(def)
  {
  }

  FormDataInput(const std::string & name, bool required) : FormDataInput<T, element>(name, required, {}) {}

  static constexpr size_t write_size_()
  {
    return 1;
  }

  void write_(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(def_);
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
struct FormArrayInput : public FormElement<FormArrayInput<T>, Elements::ArrayInput>
{
  FormArrayInput(const std::string & name, bool required, const T & def, bool fixed_size = true)
  : FormElement<FormArrayInput, Elements::ArrayInput>(name, required), def_(def), fixed_size_(fixed_size)
  {
  }

  FormArrayInput(const std::string & name, bool required, bool fixed_size = false)
  : FormArrayInput<T>(name, required, {}, fixed_size)
  {
  }

  static constexpr size_t write_size_()
  {
    return 2;
  }

  void write_(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(def_);
    builder.write(fixed_size_);
  }

  /** Invalid element */
  FormArrayInput() {}

private:
  T def_;
  bool fixed_size_;
};

struct MC_RTC_GUI_DLLAPI FormComboInput : public FormElement<FormComboInput, Elements::ComboInput>
{
  inline FormComboInput(const std::string & name,
                        bool required,
                        const std::vector<std::string> & values,
                        bool send_index = false)
  : FormElement<FormComboInput, Elements::ComboInput>(name, required), values_(values), send_index_(send_index)
  {
  }

  static constexpr size_t write_size_()
  {
    return 2;
  }

  inline void write_(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(values_);
    builder.write(send_index_);
  }

  /** Invalid element */
  inline FormComboInput() {}

private:
  std::vector<std::string> values_;
  bool send_index_ = false;
};

struct MC_RTC_GUI_DLLAPI FormDataComboInput : public FormElement<FormDataComboInput, Elements::DataComboInput>
{
  static constexpr auto type = Elements::DataComboInput;

  inline FormDataComboInput(const std::string & name,
                            bool required,
                            const std::vector<std::string> & ref,
                            bool send_index = false)
  : FormElement<FormDataComboInput, Elements::DataComboInput>(name, required), ref_(ref), send_index_(send_index)
  {
  }

  static constexpr size_t write_size_()
  {
    return 2;
  }

  inline void write_(mc_rtc::MessagePackBuilder & builder)
  {
    builder.write(ref_);
    builder.write(send_index_);
  }

  /** Invalid element */
  inline FormDataComboInput() {}

private:
  std::vector<std::string> ref_;
  bool send_index_;
};

/** Helper to create a Form element */
template<typename Callback, typename... Args>
FormImpl<Callback> Form(const std::string & name, Callback cb, Args &&... args)
{
  return FormImpl<Callback>(name, cb, std::forward<Args>(args)...);
}

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

#include "GUIState.hpp"
