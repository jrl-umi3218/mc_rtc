/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/GUIState.h>

namespace mc_rtc
{

namespace gui
{

Element::Element(const std::string & name) : name_(name) {}

StateBuilder::StateBuilder()
{
  reset();
}

void StateBuilder::reset()
{
  elements_.elements.clear();
  elements_.sub.clear();
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
    }
  }
}

size_t StateBuilder::update(std::vector<char> & buffer)
{
  mc_rtc::MessagePackBuilder builder(buffer);
  builder.start_array(3);
  builder.write(PROTOCOL_VERSION);
  if(update_data_)
  {
    data_buffer_size_ = data_.toMessagePack(data_buffer_);
    update_data_ = false;
  }
  builder.write_object(data_buffer_.data(), data_buffer_size_);
  update(builder, elements_);
  builder.finish_array();
  return builder.finish();
}

void StateBuilder::update(mc_rtc::MessagePackBuilder & builder, Category & category)
{
  builder.start_array(1 + category.elements.size() + 1);
  builder.write(category.name);
  for(auto & e : category.elements)
  {
    e.write(e.element(), builder);
  }
  builder.start_array(category.sub.size());
  for(auto & s : category.sub)
  {
    update(builder, s);
  }
  builder.finish_array();
  builder.finish_array();
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
  update_data_ = true;
  return data_;
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

} // namespace gui

} // namespace mc_rtc
