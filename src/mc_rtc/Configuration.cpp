#include <mc_rtc/Configuration.h>

#include <mc_rtc/logging.h>

#include "internals/json.h"

#include <stdexcept>
#include <fstream>

namespace mc_rtc
{

struct Configuration::Json::Impl
{
  Impl()
  : pointer(""),
    doc_p(new rapidjson::Document())
  {
  }

  Impl(const Impl & other, int idx)
  : pointer(other.pointer),
    doc_p(other.doc_p)
  {
    std::stringstream ss;
    ss << pointer << "/" << idx;
    pointer = ss.str();
  }

  Impl(const Impl & other, const std::string & key)
  : pointer(other.pointer),
    doc_p(other.doc_p)
  {
    std::stringstream ss;
    ss << pointer << "/" << key;
    pointer = ss.str();
  }

  rapidjson::Document::AllocatorType & allocator()
  {
    return doc_p->GetAllocator();
  }

  rapidjson::Value * value() const
  {
    if(pointer == "")
    {
      rapidjson::Value * ret = rapidjson::Pointer("/").Get(*doc_p);
      if(ret) { return ret; }
      else { return doc_p.get(); }
    }
    return rapidjson::Pointer(pointer.c_str()).Get(*doc_p);
  }

  bool is() const
  {
    return value() != nullptr;
  }

  bool has(const std::string & key) const
  {
    std::string fkey = pointer + "/" + key;
    rapidjson::Value * v = rapidjson::Pointer(fkey.c_str()).Get(*doc_p);
    return v != nullptr;
  }
  bool isArray() const
  {
    assert(is());
    return value()->IsArray();
  }
  bool isBool() const
  {
    assert(is());
    return value()->IsBool();
  }
  bool isInt() const
  {
    assert(is());
    return value()->IsInt();
  }
  bool isUInt() const
  {
    assert(is());
    return value()->IsUint();
  }
  bool isNumeric() const
  {
    assert(is());
    return value()->IsNumber();
  }
  bool isString() const
  {
    assert(is());
    return value()->IsString();
  }

  bool asBool() const
  {
    assert(is());
    return value()->GetBool();
  }
  int asInt() const
  {
    assert(is());
    return value()->GetInt();
  }
  unsigned int asUInt() const
  {
    assert(is());
    return value()->GetUint();
  }
  double asDouble() const
  {
    assert(is());
    return value()->GetDouble();
  }
  std::string asString() const
  {
    assert(is());
    return std::string(value()->GetString(), value()->GetStringLength());
  }

  size_t size() const
  {
    assert(is());
    return value()->Size();
  }
  Impl operator[](int idx)
  {
    return Impl(*this, idx);
  }
  Impl operator[](const std::string & key)
  {
    return Impl(*this, key);
  }

  std::string pointer;
  std::shared_ptr<rapidjson::Document> doc_p;
};

bool Configuration::Json::isArray() const
{
  return impl->isArray();
}

size_t Configuration::Json::size() const
{
  return impl->size();
}

Configuration::Json Configuration::Json::operator[](int idx) const
{
  Configuration::Json ret;
  ret.impl.reset(new Impl((*impl)[idx]));
  return ret;
}

Configuration::Json Configuration::Json::operator[](const std::string & key) const
{
  Configuration::Json ret;
  ret.impl.reset(new Impl((*impl)[key]));
  return ret;
}

Configuration::Exception::Exception(const std::string & msg)
: msg(msg)
{
}

const char * Configuration::Exception::what() const noexcept
{
  return msg.c_str();
}

Configuration::Configuration()
: v()
{
  v.impl.reset(new Json::Impl());
  v.impl->doc_p->SetObject();
}

Configuration::Configuration(const Json & v)
: v(v)
{
}

Configuration::Configuration(const char * path)
: Configuration(std::string(path))
{
}

bool Configuration::isMember(const std::string & key) const
{
  return has(key);
}

bool Configuration::has(const std::string & key) const
{
  return v.impl->has(key);
}

Configuration Configuration::operator()(const std::string & key) const
{
  if(v.impl->has(key))
  {
    return Configuration(v[key]);
  }
  throw Configuration::Exception("No entry named " + key + " in the configuration");
}

size_t Configuration::size() const
{
  if(v.impl->isArray())
  {
    return v.impl->size();
  }
  return 0;
}

Configuration Configuration::operator[](size_t i) const
{
  if(i >= size())
  {
    throw Configuration::Exception("Out-of-bound access for a Configuration element");
  }
  return Configuration(v[i]);
}

Configuration::operator bool() const
{
  if(v.impl->isBool())
  {
    return v.impl->asBool();
  }
  else if(v.impl->isUInt())
  {
    return static_cast<bool>(v.impl->asUInt());
  }
  else if(v.impl->isInt())
  {
    return static_cast<bool>(v.impl->asInt());
  }
  throw Configuration::Exception("Stored Json value is not a bool");
}

Configuration::operator int() const
{
  if(v.impl->isInt())
  {
    return v.impl->asInt();
  }
  throw Configuration::Exception("Stored Json value is not an int");
}

Configuration::operator unsigned int() const
{
  if(v.impl->isUInt() || (v.impl->isInt() && v.impl->asInt() >= 0))
  {
    return v.impl->asUInt();
  }
  throw Configuration::Exception("Stored Json value is not an unsigned int");
}

Configuration::operator double() const
{
  if(v.impl->isNumeric())
  {
    return v.impl->asDouble();
  }
  throw Configuration::Exception("Stored Json value is not a double");
}

Configuration::operator std::string() const
{
  if(v.impl->isString())
  {
    return v.impl->asString();
  }
  throw Configuration::Exception("Stored Json value is not a string");
}

Configuration::operator Eigen::Vector3d() const
{
  if(v.isArray() && v.size() == 3 && v[0].impl->isNumeric())
  {
    Eigen::Vector3d ret;
    ret << v[0].impl->asDouble(), v[1].impl->asDouble(), v[2].impl->asDouble();
    return ret;
  }
  throw Configuration::Exception("Stored Json value is not a Vector3d");
}

Configuration::operator Eigen::Vector6d() const
{
  if(v.isArray() && v.size() == 6 && v[0].impl->isNumeric())
  {
    Eigen::Vector6d ret;
    ret << v[0].impl->asDouble(), v[1].impl->asDouble(), v[2].impl->asDouble(),
           v[3].impl->asDouble(), v[4].impl->asDouble(), v[5].impl->asDouble();
    return ret;
  }
  throw Configuration::Exception("Stored Json value is not a Vector6d");
}

Configuration::operator Eigen::VectorXd() const
{
  if(v.isArray() && (v.size() == 0 || v[0].impl->isNumeric()))
  {
    Eigen::VectorXd ret(v.size());
    for(size_t i = 0; i < v.size(); ++i)
    {
      ret(i) = v[static_cast<int>(i)].impl->asDouble();
    }
    return ret;
  }
  throw Configuration::Exception("Stored Json value is not a Vector6d");
}

Configuration::operator Eigen::Quaterniond() const
{
  if(v.isArray() && v.size() == 4 && v[0].impl->isNumeric())
  {
    return Eigen::Quaterniond(v[0].impl->asDouble(), v[1].impl->asDouble(),
                              v[2].impl->asDouble(), v[3].impl->asDouble())
                             .normalized();
  }
  throw Configuration::Exception("Stored Json value is not a Quaterniond");
}

Configuration::Configuration(const std::string & path)
: v()
{
  v.impl.reset(new Json::Impl());
  load(path);
}

void Configuration::load(const std::string & path)
{
  rapidjson::Document & target = *(v.impl->doc_p);

  if(target.IsNull())
  {
    mc_rtc::internal::loadDocument(path, target);
  }
  else
  {
    rapidjson::Document d;
    if(!mc_rtc::internal::loadDocument(path, d)) { return; }
    for(auto & m : d.GetObject())
    {
      if(target.HasMember(m.name))
      {
        target.RemoveMember(m.name);
      }
      rapidjson::Value n(m.name, target.GetAllocator());
      rapidjson::Value v(m.value, target.GetAllocator());
      target.AddMember(n, v, target.GetAllocator());
    }
  }
}

void Configuration::save(const std::string & path, bool pretty)
{
  mc_rtc::internal::saveDocument(path, *v.impl->value(), pretty);
}

template<>
void Configuration::operator()(const std::string & key, std::string & v) const
{
  try
  {
    v = (std::string)(*this)(key);
  }
  catch(Exception &)
  {
  }
}

bool Configuration::operator==(const char * rhs) const
{
  return static_cast<std::string>(*this) == rhs;
}

namespace
{
  void add_impl(const std::string & key, rapidjson::Value & json,
                rapidjson::Document::AllocatorType & allocator)
  {
    rapidjson::Value key_(key.c_str(), allocator);
    rapidjson::Value value(rapidjson::kObjectType);
    if(json.HasMember(key.c_str()))
    {
      json.RemoveMember(key.c_str());
    }
    json.AddMember(key_, value, allocator);
  }

  template<typename T>
  void add_impl(const std::string & key, T value, rapidjson::Value & json,
                rapidjson::Document::AllocatorType & allocator)
  {
    rapidjson::Value key_(key.c_str(), allocator);
    rapidjson::Value value_ = mc_rtc::internal::toJSON(value, allocator);
    if(json.HasMember(key.c_str()))
    {
      json.RemoveMember(key.c_str());
    }
    json.AddMember(key_, value_, allocator);
  }

  template<typename T>
  void push_impl(T value, rapidjson::Value & json,
                rapidjson::Document::AllocatorType & allocator)
  {
    rapidjson::Value value_ = mc_rtc::internal::toJSON(value, allocator);
    if(! json.IsArray() )
    {
      throw Configuration::Exception("Trying to push data in a non-array value");
    }
    json.PushBack(value_, allocator);
  }
}

void Configuration::add(const std::string & key, bool value) { add_impl(key, value, *v.impl->value(), v.impl->allocator()); }
void Configuration::add(const std::string & key, int value) { add_impl(key, value, *v.impl->value(), v.impl->allocator()); }
void Configuration::add(const std::string & key, unsigned int value) { add_impl(key, value, *v.impl->value(), v.impl->allocator()); }
void Configuration::add(const std::string & key, double value) { add_impl(key, value, *v.impl->value(), v.impl->allocator()); }
void Configuration::add(const std::string & key, std::string value) { add_impl(key, value, *v.impl->value(), v.impl->allocator()); }
void Configuration::add(const std::string & key, Eigen::Vector3d value) { add_impl(key, value, *v.impl->value(), v.impl->allocator()); }
void Configuration::add(const std::string & key, Eigen::Vector6d value) { add_impl(key, value, *v.impl->value(), v.impl->allocator()); }
void Configuration::add(const std::string & key, Eigen::VectorXd value) { add_impl(key, value, *v.impl->value(), v.impl->allocator()); }
void Configuration::add(const std::string & key, Eigen::Quaterniond value) { add_impl(key, value, *v.impl->value(), v.impl->allocator()); }


void Configuration::add(const std::string & key, Configuration value)
{
  auto & allocator = v.impl->allocator();
  rapidjson::Value key_(key.c_str(), allocator);
  rapidjson::Value value_(*value.v.impl->value(), allocator);
  if(has(key))
  {
    v.impl->value()->RemoveMember(key.c_str());
  }
  v.impl->value()->AddMember(key_, value_, allocator);
}

Configuration Configuration::add(const std::string & key)
{
  auto & allocator = v.impl->allocator();
  rapidjson::Value key_(key.c_str(), allocator);
  rapidjson::Value value(rapidjson::kObjectType);
  if(has(key))
  {
    v.impl->value()->RemoveMember(key.c_str());
  }
  v.impl->value()->AddMember(key_, value, allocator);
  return (*this)(key);
}

Configuration Configuration::array(const std::string & key, size_t size)
{
  auto & allocator = v.impl->allocator();
  rapidjson::Value key_(key.c_str(), allocator);
  rapidjson::Value value(rapidjson::kArrayType);
  if(size) { value.Reserve(size, allocator); }
  if(has(key))
  {
    v.impl->value()->RemoveMember(key.c_str());
  }
  v.impl->value()->AddMember(key_, value, allocator);
  return (*this)(key);
}

Configuration Configuration::array(size_t reserve)
{
  if(!v.isArray())
  {
    throw(Exception("Cannot store an anonymous array outside of an array"));
  }
  auto & allocator = v.impl->allocator();
  rapidjson::Value value(rapidjson::kArrayType);
  value.Reserve(reserve, allocator);
  v.impl->value()->PushBack(value, allocator);
  return (*this)[size() - 1];
}

void Configuration::push(bool value) { push_impl(value, *v.impl->value(), v.impl->allocator()); }
void Configuration::push(int value) { push_impl(value, *v.impl->value(), v.impl->allocator()); }
void Configuration::push(unsigned int value) { push_impl(value, *v.impl->value(), v.impl->allocator()); }
void Configuration::push(double value) { push_impl(value, *v.impl->value(), v.impl->allocator()); }
void Configuration::push(std::string value) { push_impl(value, *v.impl->value(), v.impl->allocator()); }
void Configuration::push(Eigen::Vector3d value) { push_impl(value, *v.impl->value(), v.impl->allocator()); }
void Configuration::push(Eigen::Vector6d value) { push_impl(value, *v.impl->value(), v.impl->allocator()); }
void Configuration::push(Eigen::VectorXd value) { push_impl(value, *v.impl->value(), v.impl->allocator()); }
void Configuration::push(Eigen::Quaterniond value) { push_impl(value, *v.impl->value(), v.impl->allocator()); }

void Configuration::push(mc_rtc::Configuration value)
{
  auto & allocator = v.impl->allocator();
  auto & json = *v.impl->value();
  if(! json.IsArray() )
  {
    throw Configuration::Exception("Trying to push data in a non-array value");
  }
  rapidjson::Value value_(*value.v.impl->value(), allocator);
  json.PushBack(value_, allocator);
}

ConfigurationArrayIterator Configuration::begin() const
{
  return ConfigurationArrayIterator(*this);
}

ConfigurationArrayIterator Configuration::end() const
{
  auto ret = begin();
  ret.i = size();
  return ret;
}

ConfigurationArrayIterator::ConfigurationArrayIterator(const Configuration & conf)
  : i(0), conf(conf)
{
}

bool ConfigurationArrayIterator::operator!=(const ConfigurationArrayIterator & rhs) const
{
  return i != rhs.i;
}

ConfigurationArrayIterator & ConfigurationArrayIterator::operator++()
{
  ++i;
  return *this;
}

Configuration ConfigurationArrayIterator::operator*()
{
  return conf[i];
}

const Configuration ConfigurationArrayIterator::operator*() const
{
  return conf[i];
}

}

std::ostream & operator<<(std::ostream & os, const mc_rtc::Configuration & c)
{
  os << static_cast<std::string>(c);
  return os;
}
