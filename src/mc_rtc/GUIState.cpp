/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/GUIState.h>

namespace mc_rtc
{

namespace gui
{

Element::Element(const std::string & name) : name_(name) {}

FormComboInput::FormComboInput(const std::string & name,
                               bool required,
                               const std::vector<std::string> & values,
                               bool send_index)
: FormInput<FormComboInput>(name, required), values_(values), send_index_(send_index)
{
}

void FormComboInput::addGUI_(mc_rtc::Configuration & gui)
{
  gui.add("values", values_);
  if(send_index_)
  {
    gui.add("send_index", true);
  }
}

FormDataComboInput::FormDataComboInput(const std::string & name,
                                       bool required,
                                       const std::vector<std::string> & ref,
                                       bool send_index)
: FormInput<FormDataComboInput>(name, required), ref_(ref), send_index_(send_index)
{
}

void FormDataComboInput::addGUI_(mc_rtc::Configuration & gui)
{
  gui.add("ref", ref_);
  if(send_index_)
  {
    gui.add("send_index", true);
  }
}

StateBuilder::StateBuilder()
{
  reset();
}

void StateBuilder::reset()
{
  elements_.elements.clear();
  elements_.sub.clear();
  state_ = mc_rtc::Configuration{};
  state_.add("DATA"); // static Data of the GUI
  state_.add("GUI"); // GUI description of the GUI
  state_.add("STATE"); // State data of the GUI
}

std::string StateBuilder::cat2str(const std::vector<std::string> & cat)
{
  std::string ret;
  for(size_t i = 0; i < cat.size(); ++i)
  {
    ret += cat[i];
    if(i != cat.size() - 1)
    {
      ret += "/";
    }
  }
  return ret;
}

void StateBuilder::removeCategory(const std::vector<std::string> & category)
{
  if(category.size() == 0)
  {
    LOG_ERROR("You are not allowed to remove the root of the state")
    LOG_WARNING("Call clear() if this was your intent")
    return;
  }
  std::pair<bool, Category &> cat = getCategory(category, true);
  if(cat.first)
  {
    auto it = cat.second.find(category.back());
    if(it == cat.second.sub.end())
    {
      return;
    }
    cat.second.sub.erase(it);
    auto s = state_("STATE");
    auto g = state_("GUI");
    for(size_t i = 0; i < category.size() - 1; ++i)
    {
      const auto & c = category[i];
      if(!s.has("_sub") || !s("_sub").has(c)) return;
      s = s("_sub")(c);
      g = g("_sub")(c);
    }
    if(!s.has("_sub") || !s("_sub").has(category.back()))
    {
      return;
    }
    s("_sub").remove(category.back());
    g("_sub").remove(category.back());
    if(s("_sub").empty())
    {
      s.remove("_sub");
      g.remove("_sub");
    }
  }
}

void StateBuilder::removeElement(const std::vector<std::string> & category, const std::string & name)
{
  bool found;
  std::reference_wrapper<Category> cat_(elements_);
  std::tie(found, cat_) = getCategory(category, false);
  Category & cat = cat_;
  if(found)
  {
    auto it = std::find_if(cat.elements.begin(), cat.elements.end(),
                           [&name](const ElementStore & el) { return el().name() == name; });
    if(it != cat.elements.end())
    {
      cat.elements.erase(it);
      auto s = state_("STATE");
      auto g = state_("GUI");
      for(const auto & c : category)
      {
        if(!s.has("_sub") || !s("_sub").has(c)) return;
        s = s("_sub")(c);
        g = g("_sub")(c);
      }
      if(s.has(name))
      {
        auto g_details = g(name);
        if(!g_details.has("_sub"))
        {
          s.remove(name);
          g.remove(name);
          return;
        }
        auto g_keys = g_details.keys();
        for(const auto & k : g_keys)
        {
          if(k != "_sub")
          {
            g_details.remove(k);
          }
        }
        auto s_details = s(name);
        auto s_keys = s_details.keys();
        for(const auto & k : s_keys)
        {
          if(k != "_sub")
          {
            s_details.remove(k);
          }
        }
      }
    }
  }
}

const mc_rtc::Configuration & StateBuilder::update()
{
  update(elements_, state_("STATE"));
  return state_;
}

void StateBuilder::update(Category & category, mc_rtc::Configuration out)
{
  for(auto & e : category.elements)
  {
    auto & element = e();
    if(!out.has(element.name()))
    {
      out.add(element.name());
    }
    auto c = out(element.name());
    e.addData(element, c);
  }
  if(!category.sub.size())
  {
    return;
  }
  if(!out.has("_sub"))
  {
    out.add("_sub");
  }
  out = out("_sub");
  for(auto & c : category.sub)
  {
    if(!out.has(c.name))
    {
      out.add(c.name);
    }
    update(c, out(c.name));
  }
}

bool StateBuilder::handleRequest(const std::vector<std::string> & category,
                                 const std::string & name,
                                 const mc_rtc::Configuration & data)
{
  bool found;
  std::reference_wrapper<Category> cat_(elements_);
  std::tie(found, cat_) = getCategory(category, false);
  if(!found)
  {
    LOG_ERROR("No category " << cat2str(category))
    return false;
  }
  Category & cat = cat_;
  auto it = std::find_if(cat.elements.begin(), cat.elements.end(),
                         [&name](const ElementStore & el) { return el().name() == name; });
  if(it == cat.elements.end())
  {
    LOG_ERROR("No element " << name << " in category " << cat2str(category))
    return false;
  }
  ElementStore & el = *it;
  Element & elem = el();
  try
  {
    return el.handleRequest(elem, data);
  }
  catch(const mc_rtc::Configuration::Exception & exc)
  {
    LOG_ERROR("Failed to handle request for " << cat2str(category) << "/" << name << "\n" << exc.what())
    LOG_WARNING(data.dump(true))
    return false;
  }
}

mc_rtc::Configuration StateBuilder::data()
{
  return state_("DATA");
}

std::pair<bool, StateBuilder::Category &> StateBuilder::getCategory(const std::vector<std::string> & category,
                                                                    bool getParent)
{
  std::reference_wrapper<Category> cat_(elements_);
  size_t limit = category.size();
  if(getParent)
  {
    assert(limit > 0);
    limit -= 1;
  }
  for(size_t i = 0; i < limit; ++i)
  {
    const auto & c = category[i];
    Category & cat = cat_;
    auto it = cat.find(c);
    if(it == cat.sub.end())
    {
      return {false, cat_};
    }
    cat_ = *it;
  }
  return {true, cat_};
}

StateBuilder::Category & StateBuilder::getCategory(const std::vector<std::string> & category)
{
  std::reference_wrapper<Category> cat_(elements_);
  for(const auto & c : category)
  {
    auto & cat = cat_.get();
    auto it = cat.find(c);
    if(it == cat.sub.end())
    {
      cat.sub.push_back({c, {}, {}, 0});
      it = std::prev(cat.sub.end());
    }
    cat_ = *it;
  }
  return cat_;
}

const Element & StateBuilder::ElementStore::operator()() const
{
  return element();
}
Element & StateBuilder::ElementStore::operator()()
{
  return element();
}

std::vector<StateBuilder::Category>::iterator StateBuilder::Category::find(const std::string & name)
{
  for(auto it = sub.begin(); it != sub.end(); ++it)
  {
    if(name == it->name)
    {
      return it;
    }
  }
  return sub.end();
}

void StateBuilder::updateGUI(const std::vector<std::string> & category, StateBuilder::ElementStore & el)
{
  auto gui = state_("GUI");
  for(const auto & c : category)
  {
    if(!gui.has("_sub"))
    {
      gui.add("_sub");
    }
    gui = gui("_sub");
    if(!gui.has(c))
    {
      gui.add(c);
    }
    gui = gui(c);
  }
  auto & element = el();
  if(!gui.has(element.name()))
  {
    gui.add(element.name());
  }
  gui = gui(element.name());
  el.addGUI(element, gui);
}

} // namespace gui

} // namespace mc_rtc
