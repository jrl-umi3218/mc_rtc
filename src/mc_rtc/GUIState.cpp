#include <mc_rtc/GUIState.h>

namespace mc_rtc
{

namespace gui
{

Element<void>::Element(const std::vector<std::string> & names, set_fn_t set_fn)
: set_fn_(set_fn)
{
  if(!names.size())
  {
    throw std::runtime_error("Cannot add an element without names");
  }
  if(names.size() > 1)
  {
    categories_.resize(names.size() - 1);
    for(size_t i = 0; i < names.size() - 1; ++i)
    {
      categories_[i] = names[i];
    }
  }
  name_ = names.back();
}

bool Element<void>::handleRequest(const mc_rtc::Configuration &) const
{
  set_fn_();
  return true;
}

Label::Label(bool is_vector)
: is_vector(is_vector)
{
}

void Label::addData(mc_rtc::Configuration out) const
{
  if(is_vector)
  {
    out.add("vector", is_vector);
  }
}

Schema::Schema(const std::string & schema_dir)
: schema_dir_(schema_dir)
{
}

void Schema::addData(mc_rtc::Configuration out) const
{
  out.add("schema_dir", schema_dir_);
}

mc_rtc::Configuration State::getStateCategory(const std::vector<std::string> & category)
{
  return getCategory(state, category);
}

mc_rtc::Configuration State::getCategory(mc_rtc::Configuration & config, const std::vector<std::string> & category)
{
  auto ret = config;
  for(const auto & c : category)
  {
    if(ret.has(c))
    {
      ret = ret(c);
    }
    else
    {
      ret = ret.add(c);
    }
  }
  return ret;
}

void StateBuilder::reset()
{
  elements_.clear();
  state_.state = mc_rtc::Configuration();
}

namespace
{
  // Returns true if cat is a sub-category of base
  bool is_sub_category(const std::vector<std::string> & cat,
                       const std::vector<std::string> & base)
  {
    bool ret = cat.size() > base.size();
    size_t i = 0;
    while(ret && i < base.size())
    {
      ret = ret && base[i] == cat[i];
      ++i;
    }
    return ret;
  }
}

void StateBuilder::removeCategory(const std::vector<std::string> & category)
{
  // Don't allow removing the root
  if(category.size() == 0)
  {
    LOG_ERROR("You are not allowed to remove the root of State")
    LOG_WARNING("Call reset() if this was your intent")
    return;
  }
  auto cat = elements_.find(category);
  if(cat != elements_.end())
  {
    elements_.erase(cat);
    if(category.size() > 1)
    {
      if(state_.state.has(category[0]))
      {
        auto out = state_.state(category[0]);
        for(size_t i = 1; i < category.size() - 1; ++i)
        {
          out = out(category[i]);
        }
        out.remove(category.back());
      }
    }
    else
    {
      state_.state.remove(category[0]);
    }
  }
  // Remove sub-categories
  for(auto it = elements_.begin(); it != elements_.end();)
  {
    if(is_sub_category(it->first, category))
    {
      it = elements_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void StateBuilder::removeElement(const std::vector<std::string> & names)
{
  if(names.size() == 0) { return; }
  std::vector<std::string> category(names.size() - 1);
  if(names.size() > 1)
  {
    std::copy(names.begin(), names.end(), category.begin());
  }
  auto cat = elements_.find(category);
  if(cat != elements_.end())
  {
    const auto & name = names.back();
    auto el = cat->second.find(name);
    if(el != cat->second.end())
    {
      // Remove the callback
      cat->second.erase(el);
      // Remove the element from the output
      if(category.size())
      {
        auto out = state_.state(category[0]);
        for(size_t i = 1; i < category.size(); ++i)
        {
          out = out(category[i]);
        }
        out.remove(name);
      }
      else
      {
        state_.state.remove(name);
      }
    }
    // If the category is now empty, remove it as well
    if(cat->second.size() == 0)
    {
      removeCategory(category);
    }
  }
}

const State & StateBuilder::updateState()
{
  for(const auto & el : elements_)
  {
    auto cat = state_.getStateCategory(el.first);
    for(const auto & e : el.second)
    {
      e.second(cat);
    }
  }
  return state_;
}

bool StateBuilder::callMethod(const mc_rtc::Configuration & method)
{
  auto category_in = method("category", std::vector<std::string>{});
  if(methods_.count(category_in))
  {
    const auto & category = methods_.at(category_in);
    auto name = method("name", std::string{});
    if(category.count(name))
    {
      return category.at(name)(method);
    }
    else
    {
      LOG_ERROR("No property named " << name << " in requested category")
      return false;
    }
  }
  else
  {
    LOG_ERROR("Attempted to call a method on non-existing category")
    return false;
  }
}

} // namespace gui

} // namepsace mc_rtc
