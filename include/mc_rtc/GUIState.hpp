#pragma once

#include <mc_rtc/GUIState.h>

namespace mc_rtc
{

namespace gui
{

template<typename GetT>
DataElement<GetT>::DataElement(const std::string & name, GetT get_fn)
: Element(name), get_fn_(get_fn)
{
  name_ = name;
}

template<typename GetT>
void DataElement<GetT>::addData(mc_rtc::Configuration & data)
{
  data.add("data", get_fn_());
}

template<typename ElementT, typename Callback>
template<typename ... Args>
CallbackElement<ElementT, Callback>::CallbackElement(const std::string & name, Callback cb, Args && ... args)
: ElementT(name, std::forward<Args>(args)...), cb_(cb)
{
}

template<typename ElementT, typename Callback>
bool CallbackElement<ElementT, Callback>::handleRequest(const mc_rtc::Configuration & data)
{
  cb_(data);
  return true;
}

template<typename ElementT, typename Callback>
bool VoidCallbackElement<ElementT, Callback>::handleRequest(const mc_rtc::Configuration &)
{
  this->cb_();
  return true;
}

template<typename GetT>
LabelImpl<GetT>::LabelImpl(const std::string & name, GetT get_fn)
: DataElement<GetT>(name, get_fn)
{
}

template<typename GetT>
ArrayLabelImpl<GetT>::ArrayLabelImpl(const std::string & name,
                                     const std::vector<std::string> & labels,
                                     GetT get_fn)
: LabelImpl<GetT>(name, get_fn),
  labels_(labels)
{
}

template<typename GetT>
void ArrayLabelImpl<GetT>::addGUI(mc_rtc::Configuration & gui)
{
  if(labels_.size()) { gui.add("labels", labels_); }
}

template<typename GetT, typename Callback>
CheckboxImpl<GetT, Callback>::CheckboxImpl(const std::string & name,
                                           GetT get_fn, Callback cb)
: VoidCallbackElement<DataElement<GetT>, Callback>(name, cb, get_fn)
{
}

template<typename GetT, typename SetT>
CommonInputImpl<GetT, SetT>::CommonInputImpl(const std::string & name, GetT get_fn, SetT set_fn)
: CallbackElement<DataElement<GetT>, SetT>(name, set_fn, get_fn)
{
}

template<typename GetT, typename SetT>
void NumberSliderImpl<GetT, SetT>::addGUI(mc_rtc::Configuration & gui)
{
  gui.add("min", min_);
  gui.add("max", max_);
}

template<typename GetT, typename SetT>
ArrayInputImpl<GetT, SetT>::ArrayInputImpl(const std::string & name,
                                           const std::vector<std::string> & labels,
                                           GetT get_fn, SetT set_fn)
: ArrayInputImpl(name, get_fn, set_fn)
{
  labels_ = labels;
}

template<typename GetT, typename SetT>
void ArrayInputImpl<GetT, SetT>::addGUI(mc_rtc::Configuration & out)
{
  if(labels_.size()) { out.add("labels", labels_); }
}

template<typename GetT, typename SetT>
ComboInputImpl<GetT, SetT>::ComboInputImpl(const std::string & name,
                                           const std::vector<std::string> & values,
                                           GetT get_fn, SetT set_fn)
: CommonInputImpl<GetT, SetT>(name, get_fn, set_fn),
  values_(values)
{
}

template<typename GetT, typename SetT>
void ComboInputImpl<GetT, SetT>::addGUI(mc_rtc::Configuration & out)
{
  out.add("values", values_);
}

template<typename GetT>
void Point3DROImpl<GetT>::addGUI(mc_rtc::Configuration & gui)
{
  gui.add("ro", true);
}

template<typename GetT>
DisplayPoint3DTrajectoryImpl<GetT>::DisplayPoint3DTrajectoryImpl(const std::string & name, GetT get_fn)
: DataElement<GetT>(name, get_fn)
{
}

template<typename GetT>
DisplayPoseTrajectoryImpl<GetT>::DisplayPoseTrajectoryImpl(const std::string & name, GetT get_fn)
: DataElement<GetT>(name, get_fn)
{
}

template<typename GetT>
DisplayPolygonImpl<GetT>::DisplayPolygonImpl(const std::string & name, GetT get_fn)
: DataElement<GetT>(name, get_fn)
{
}

template<typename GetT>
void RotationROImpl<GetT>::addGUI(mc_rtc::Configuration & gui)
{
  gui.add("ro", true);
}

template<typename GetT>
void TransformROImpl<GetT>::addGUI(mc_rtc::Configuration & gui)
{
  gui.add("ro", true);
}

template<typename GetT, typename SetT>
DataComboInputImpl<GetT, SetT>::DataComboInputImpl(const std::string & name,
                                           const std::vector<std::string> & values,
                                           GetT get_fn, SetT set_fn)
: CommonInputImpl<GetT, SetT>(name, get_fn, set_fn),
  data_ref_(values)
{
}

template<typename GetT, typename SetT>
void DataComboInputImpl<GetT, SetT>::addGUI(mc_rtc::Configuration & out)
{
  out.add("ref", data_ref_);
}

template<typename Callback>
SchemaImpl<Callback>::SchemaImpl(const std::string & name,
                                 const std::string & schema,
                                 Callback cb)
: CallbackElement<Element,Callback>(name, cb),
  schema_(schema)
{
}

template<typename Callback>
void SchemaImpl<Callback>::addGUI(mc_rtc::Configuration & gui)
{
  gui.add("dir", schema_);
}

template<typename Derived>
FormInput<Derived>::FormInput(const std::string & name, bool required)
: name_(name), required_(required)
{
}

template<typename Derived>
void FormInput<Derived>::addGUI(mc_rtc::Configuration & gui)
{
  auto el = gui.add(name_);
  if(required_) { el.add("required", required_); }
  el.add("type", static_cast<int>(Derived::type));
  static_cast<Derived*>(this)->addGUI_(el);
}

namespace
{

template<typename ... Args>
void FormImpl_MakeGUIData(mc_rtc::Configuration &)
{
}

template<typename Arg0, typename ... Args>
void FormImpl_MakeGUIData(mc_rtc::Configuration & gui, Arg0 && arg, Args && ... args)
{
  arg.addGUI(gui);
  FormImpl_MakeGUIData(gui, std::forward<Args>(args)...);
}

}

template<typename Callback>
template<typename ... Args>
FormImpl<Callback>::FormImpl(const std::string & name, Callback cb, Args && ... args)
: CallbackElement<Element, Callback>(name, cb)
{
  FormImpl_MakeGUIData(gui_, std::forward<Args>(args)...);
}

template<typename Callback>
void FormImpl<Callback>::addGUI(mc_rtc::Configuration & gui)
{
  gui.add("form", gui_);
}

template<typename Callback>
template<typename T>
void FormImpl<Callback>::addElement(T && arg)
{
  arg.addGUI(gui_);
}

template<typename T>
void StateBuilder::addElement(const std::vector<std::string> & category, T element)
{
  static_assert(std::is_base_of<Element, T>::value,
                "You can only add elements that derive from the Element class");
  Category & cat = getCategory(category);
  auto it = std::find_if(cat.elements.begin(),
                         cat.elements.end(),
                         [&element](const ElementStore & el)
                         {
                         return el().name() == element.name();
                         });
  if(it != cat.elements.end())
  {
    LOG_ERROR("An element named " << element.name() << " already exists in " << cat2str(category))
    LOG_WARNING("Discarding request to add this element")
    return;
  }
  cat.elements.emplace_back(element);
}

template<typename T, typename ... Args>
void StateBuilder::addElement(const std::vector<std::string> & category,
                              T element, Args ... args)
{
  addElement(category, element);
  addElement(category, args...);
}


template<typename T>
StateBuilder::ElementStore::ElementStore(T self)
{
  element = [self]() mutable -> Element & { return self; };
  addData = [](Element & el, mc_rtc::Configuration & out)
            {
              static_cast<T&>(el).addData(out);
            };
  addGUI = [](Element & el, mc_rtc::Configuration & out)
           {
             out.add("type", static_cast<int>(T::type));
             static_cast<T&>(el).addGUI(out);
           };
  handleRequest = [](Element & el, const mc_rtc::Configuration & data)
                  {
                    T el_ = static_cast<T&>(el);
                    return el_.handleRequest(data);
                  };
}

} // namespace gui

} // namepsace mc_rtc
