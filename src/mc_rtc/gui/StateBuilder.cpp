/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/gui/StateBuilder.h>

#include <mc_rtc/gui/plot/types.h>

namespace mc_rtc
{

namespace gui
{

// Repeat static constexpr declarations
// See https://stackoverflow.com/q/8016780
constexpr int8_t StateBuilder::PROTOCOL_VERSION;

const Color Color::White = Color(1, 1, 1, 1);
const Color Color::Black = Color(0, 0, 0, 1);
const Color Color::Red = Color(1, 0, 0, 1);
const Color Color::Green = Color(0, 1, 0, 1);
const Color Color::Blue = Color(0, 0, 1, 1);
const Color Color::Cyan = Color(0, 1, 1, 1);
const Color Color::Magenta = Color(1, 0, 1, 1);
const Color Color::Yellow = Color(1, 1, 0, 1);
const Color Color::Gray = Color(0.6, 0.6, 0.6, 1);
const Color Color::LightGray = Color(0.75, 0.75, 0.75, 1);
const std::map<std::string, Color> Color::ColorMap{
    {"white", Color::White}, {"black", Color::Black},         {"red", Color::Red},         {"green", Color::Green},
    {"blue", Color::Blue},   {"cyan", Color::Cyan},           {"magenta", Color::Magenta}, {"yellow", Color::Yellow},
    {"gray", Color::Gray},   {"lightgray", Color::LightGray},
};

namespace plot
{

constexpr double Range::inf;

} // namespace plot

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

void StateBuilder::removePlot(const std::string & name)
{
  auto it = plots_.find(name);
  if(it != plots_.end())
  {
    plots_.erase(it);
  }
}

void StateBuilder::removeCategory(const std::vector<std::string> & category)
{
  if(category.size() == 0)
  {
    mc_rtc::log::error("You are not allowed to remove the root of the state");
    mc_rtc::log::warning("Call clear() if this was your intent");
    return;
  }
  size_t depth = category.size() - 1;
  auto cat = getCategory(category, depth);
  while(cat)
  {
    auto it = cat->find(category[depth]);
    if(it == cat->sub.end())
    {
      return;
    }
    cat->sub.erase(it);
    if(cat->elements.size() == 0 && cat->sub.size() == 0 && depth > 0)
    {
      depth -= 1;
      cat = getCategory(category, depth);
    }
    else
    {
      cat = nullptr;
    }
  }
}

bool StateBuilder::hasElement(const std::vector<std::string> & category, const std::string & name)
{
  auto cat_ = getCategory(category);
  if(!cat_)
  {
    return false;
  }
  const auto & cat = *cat_;
  auto it = std::find_if(cat.elements.begin(), cat.elements.end(),
                         [&name](const ElementStore & el) { return el().name() == name; });
  return it != cat.elements.end();
}

void StateBuilder::removeElement(const std::vector<std::string> & category, const std::string & name)
{
  auto cat_ = getCategory(category);
  if(!cat_)
  {
    return;
  }
  auto & cat = *cat_;
  auto it = std::find_if(cat.elements.begin(), cat.elements.end(),
                         [&name](const ElementStore & el) { return el().name() == name; });
  if(it != cat.elements.end())
  {
    cat.elements.erase(it);
  }
  if(cat.elements.size() == 0 && cat.sub.size() == 0)
  {
    removeCategory(category);
  }
}

void StateBuilder::removeElements(const std::vector<std::string> & category, void * source, bool recurse)
{
  if(source == nullptr)
  {
    return;
  }
  auto cat_ = getCategory(category);
  if(!cat_)
  {
    return;
  }
  auto & elements = cat_->elements;
  if(recurse)
  {
    removeElements(*cat_, source);
  }
  else
  {
    elements.erase(std::remove_if(elements.begin(), elements.end(),
                                  [source](const ElementStore & elem) { return elem.source == source; }),
                   elements.end());
  }
  if(elements.size() == 0 && cat_->sub.size() == 0)
  {
    removeCategory(category);
  }
}

void StateBuilder::removeElements(void * source)
{
  if(source == nullptr)
  {
    return;
  }
  removeElements(elements_, source);
}

void StateBuilder::removeElements(Category & category, void * source)
{
  auto & sub = category.sub;
  sub.erase(std::remove_if(sub.begin(), sub.end(),
                           [this, source](Category & cat) {
                             removeElements(cat, source);
                             return cat.elements.size() == 0 && cat.sub.size() == 0;
                           }),
            sub.end());
  auto & elements = category.elements;
  elements.erase(std::remove_if(elements.begin(), elements.end(),
                                [source](const ElementStore & elem) { return elem.source == source; }),
                 elements.end());
}

size_t StateBuilder::update(std::vector<char> & buffer)
{
  mc_rtc::MessagePackBuilder builder(buffer);
  builder.start_array(4);

  // Write protocol version
  builder.write(PROTOCOL_VERSION);

  // Write static data
  if(update_data_)
  {
    data_buffer_size_ = data_.toMessagePack(data_buffer_);
    update_data_ = false;
  }
  builder.write_object(data_buffer_.data(), data_buffer_size_);

  // Write elements
  update(builder, elements_);

  // Write plots
  builder.start_array(plots_.size());
  for(auto & p : plots_)
  {
    builder.start_array(p.second.msg_size);
    p.second.callback(builder, p.first);
    builder.finish_array();
  }
  builder.finish_array();

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
  auto cat_ = getCategory(category);
  if(!cat_)
  {
    mc_rtc::log::error("No category {}", cat2str(category));
    return false;
  }
  Category & cat = *cat_;
  auto it = std::find_if(cat.elements.begin(), cat.elements.end(),
                         [&name](const ElementStore & el) { return el().name() == name; });
  if(it == cat.elements.end())
  {
    mc_rtc::log::error("No element {} in category {}", name, cat2str(category));
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
    mc_rtc::log::error("Failed to handle request for {}/{}\n{}", cat2str(category), name, exc.what());
    mc_rtc::log::warning(data.dump(true));
    return false;
  }
}

mc_rtc::Configuration StateBuilder::data()
{
  update_data_ = true;
  return data_;
}

auto StateBuilder::getCategory(const std::vector<std::string> & category, size_t depth) -> Category *
{
  Category * cat_ = &elements_;
  size_t limit = std::min(depth, category.size());
  for(size_t i = 0; i < limit; ++i)
  {
    const auto & c = category[i];
    auto it = cat_->find(c);
    if(it == cat_->sub.end())
    {
      return nullptr;
    }
    cat_ = &(*it);
  }
  return cat_;
}

StateBuilder::Category & StateBuilder::getOrCreateCategory(const std::vector<std::string> & category)
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
