/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/MessagePackBuilder.h>
#include <mc_rtc/gui/api.h>

/** This header contains the base block of all elements in the GUI system. For the implementation details of specific
 * elements look into their dedicated header */

namespace mc_rtc
{

namespace gui
{

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
  XYTheta,
  Table,
  Robot,
  Visual
};

/** Element is the common class for every element's type available in the
 * GUI. Since we use static polymorphism in the \ref StateBuilder we don't
 * need to make any functions virtual */
struct MC_RTC_GUI_DLLAPI Element
{
  /** Name of the element */
  const std::string & name() const
  {
    return name_;
  }

  /** Access the stack id of an element (also referred to as SID)
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
   * By default, it includes the widget name, type and SID.
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

/** Saves typing for the most common case */
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

} // namespace gui

} // namespace mc_rtc
