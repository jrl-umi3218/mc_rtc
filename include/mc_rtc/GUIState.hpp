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
